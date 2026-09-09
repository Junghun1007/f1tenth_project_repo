"""Fast-SCNN HighRes architecture required by the bundled checkpoint."""

import torch
from torch import nn
from torch.nn import functional as functional


ARCHITECTURE = "fast_scnn_highres_v1"


class ConvBNReLU(nn.Sequential):
    def __init__(
        self,
        inputs,
        outputs,
        kernel=3,
        stride=1,
        groups=1,
        dilation=1,
    ):
        super().__init__(
            nn.Conv2d(
                inputs,
                outputs,
                kernel,
                stride,
                dilation * (kernel // 2),
                groups=groups,
                dilation=dilation,
                bias=False,
            ),
            nn.BatchNorm2d(outputs),
            nn.ReLU(inplace=True),
        )


class DSConv(nn.Sequential):
    def __init__(self, inputs, outputs, stride=1):
        super().__init__(
            nn.Conv2d(
                inputs,
                inputs,
                3,
                stride,
                1,
                groups=inputs,
                bias=False,
            ),
            nn.BatchNorm2d(inputs),
            ConvBNReLU(inputs, outputs, kernel=1),
        )


class Bottleneck(nn.Module):
    def __init__(self, inputs, outputs, stride):
        super().__init__()
        hidden = inputs * 6
        self.body = nn.Sequential(
            ConvBNReLU(inputs, hidden, kernel=1),
            ConvBNReLU(hidden, hidden, stride=stride, groups=hidden),
            nn.Conv2d(hidden, outputs, 1, bias=False),
            nn.BatchNorm2d(outputs),
        )
        self.residual = stride == 1 and inputs == outputs

    def forward(self, tensor):
        output = self.body(tensor)
        return output + tensor if self.residual else output


class PyramidPooling(nn.Module):
    def __init__(self, channels=128):
        super().__init__()
        self.bins = (1, 2, 3, 6)
        self.branches = nn.ModuleList(
            [
                nn.Sequential(
                    nn.Conv2d(channels, channels // 4, 1, bias=False),
                    nn.GroupNorm(4, channels // 4),
                    nn.ReLU(inplace=True),
                )
                for _ in self.bins
            ]
        )
        self.project = ConvBNReLU(channels * 2, channels, kernel=1)

    def forward(self, tensor):
        size = tensor.shape[-2:]
        features = [tensor]
        for bins, branch in zip(self.bins, self.branches):
            pooled = functional.adaptive_avg_pool2d(
                tensor,
                (min(bins, size[0]), min(bins, size[1])),
            )
            features.append(
                functional.interpolate(
                    branch(pooled),
                    size=size,
                    mode="bilinear",
                    align_corners=False,
                )
            )
        return self.project(torch.cat(features, dim=1))


class FeatureFusion(nn.Module):
    def __init__(self):
        super().__init__()
        self.low_depthwise = ConvBNReLU(
            128,
            128,
            groups=128,
            dilation=4,
        )
        self.low_project = nn.Sequential(
            nn.Conv2d(128, 128, 1, bias=False),
            nn.BatchNorm2d(128),
        )
        self.high_project = nn.Sequential(
            nn.Conv2d(64, 128, 1, bias=False),
            nn.BatchNorm2d(128),
        )

    def forward(self, high, low):
        low = functional.interpolate(
            low,
            size=high.shape[-2:],
            mode="bilinear",
            align_corners=False,
        )
        return functional.relu(
            self.high_project(high) +
            self.low_project(self.low_depthwise(low)),
            inplace=True,
        )


class FastSCNN(nn.Module):
    """RGB Bx3xHxW in [0,1] to independent left/right logits."""

    def __init__(self, auxiliary=True):
        super().__init__()
        self.downsample = nn.Sequential(
            ConvBNReLU(3, 32, stride=2),
            DSConv(32, 48, 2),
            DSConv(48, 64, 2),
        )
        blocks = []
        inputs = 64
        for outputs, stride in ((64, 2), (96, 2), (128, 1)):
            blocks.extend(
                [
                    Bottleneck(inputs, outputs, stride),
                    Bottleneck(outputs, outputs, 1),
                    Bottleneck(outputs, outputs, 1),
                ]
            )
            inputs = outputs
        self.global_features = nn.Sequential(*blocks, PyramidPooling())
        self.fusion = FeatureFusion()
        self.classifier = nn.Sequential(
            DSConv(128, 128),
            DSConv(128, 128),
            nn.Dropout2d(0.1),
            nn.Conv2d(128, 2, 1),
        )
        self.auxiliary = (
            nn.Sequential(
                ConvBNReLU(64, 32),
                nn.Dropout2d(0.1),
                nn.Conv2d(32, 2, 1),
            )
            if auxiliary
            else None
        )

    def forward(self, tensor):
        size = tensor.shape[-2:]
        high = self.downsample(tensor)
        fused = self.fusion(high, self.global_features(high))
        result = {
            "out": functional.interpolate(
                self.classifier(fused),
                size=size,
                mode="bilinear",
                align_corners=False,
            )
        }
        if self.training and self.auxiliary is not None:
            result["aux"] = functional.interpolate(
                self.auxiliary(high),
                size=size,
                mode="bilinear",
                align_corners=False,
            )
        return result


class FastSCNNHighRes(FastSCNN):
    """Fast-SCNN with learned 1/4 and 1/2 resolution skip decoding."""

    def __init__(self, auxiliary=True):
        super().__init__(auxiliary=auxiliary)
        del self.classifier
        self.decoder_project = ConvBNReLU(128, 64, kernel=1)
        self.decoder_quarter = nn.Sequential(
            DSConv(64 + 48, 64),
            DSConv(64, 64),
        )
        self.decoder_half = nn.Sequential(
            DSConv(64 + 32, 32),
            DSConv(32, 32),
        )
        self.decoder_classifier = nn.Sequential(
            nn.Dropout2d(0.1),
            nn.Conv2d(32, 2, 1),
        )

    def forward(self, tensor):
        size = tensor.shape[-2:]
        half = self.downsample[0](tensor)
        quarter = self.downsample[1](half)
        eighth = self.downsample[2](quarter)
        fused = self.fusion(eighth, self.global_features(eighth))
        decoded = functional.interpolate(
            self.decoder_project(fused),
            size=quarter.shape[-2:],
            mode="bilinear",
            align_corners=False,
        )
        decoded = self.decoder_quarter(torch.cat((decoded, quarter), dim=1))
        decoded = functional.interpolate(
            decoded,
            size=half.shape[-2:],
            mode="bilinear",
            align_corners=False,
        )
        decoded = self.decoder_half(torch.cat((decoded, half), dim=1))
        result = {
            "out": functional.interpolate(
                self.decoder_classifier(decoded),
                size=size,
                mode="bilinear",
                align_corners=False,
            )
        }
        if self.training and self.auxiliary is not None:
            result["aux"] = functional.interpolate(
                self.auxiliary(eighth),
                size=size,
                mode="bilinear",
                align_corners=False,
            )
        return result


def validate_checkpoint(checkpoint):
    if checkpoint.get("architecture") != ARCHITECTURE:
        raise ValueError(
            "Expected fast_scnn_highres_v1 checkpoint, got {!r}".format(
                checkpoint.get("architecture")
            )
        )
