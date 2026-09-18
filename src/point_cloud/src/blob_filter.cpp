#include "point_cloud/blob_filter.hpp"

#include <opencv2/imgproc.hpp>

namespace point_cloud
{
BlobResult filterBlobs(const ClusterResult & input, const BlobOptions & o, const ClusterOptions & c)
{
  validateBlobs(o);
  validateClusters(c);
  if (input.points.xyz.size() != input.ids.size()*3) {
    throw std::invalid_argument("Blob input labels do not match XYZ points");
  }
  if (!o.enabled) {return {input,0};}
  BlobResult output;
  output.clusters.points.height = 1;
  std::unordered_map<std::uint32_t,std::size_t> group_index;
  std::vector<std::vector<std::size_t>> groups;
  for (std::size_t i=0; i<input.ids.size(); ++i) {
    if (input.ids[i]==0) {continue;}
    const auto entry = group_index.emplace(input.ids[i],groups.size());
    if (entry.second) {groups.emplace_back();}
    groups[entry.first->second].push_back(i);
  }
  std::vector<std::uint32_t> output_labels(input.ids.size(),0);
  const int padding = 2*(o.closing_radius_cells+o.opening_radius_cells)+2;
  for (const auto & group : groups) {
    struct Pixel {std::size_t index; int x,y;};
    std::vector<Pixel> pixels;
    int min_x=1000000000, min_y=min_x, max_x=-min_x, max_y=-min_x;
    for (auto i : group) {
      const double x=input.points.xyz[i*3], y=input.points.xyz[i*3+1], z=input.points.xyz[i*3+2];
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {continue;}
      const double gx=std::floor(x/o.cell_size_m), gy=std::floor(y/o.cell_size_m);
      if (std::abs(gx)>1e9 || std::abs(gy)>1e9) {continue;}
      const int px=static_cast<int>(gx), py=static_cast<int>(gy);
      pixels.push_back({i,px,py});
      min_x=std::min(min_x,px); max_x=std::max(max_x,px);
      min_y=std::min(min_y,py); max_y=std::max(max_y,py);
    }
    if (pixels.empty()) {continue;}
    const std::int64_t width=static_cast<std::int64_t>(max_x)-min_x+1+2*padding;
    const std::int64_t height=static_cast<std::int64_t>(max_y)-min_y+1+2*padding;
    // Bound memory even when BEV cropping is disabled or an outlier is distant.
    constexpr std::int64_t max_cells=1000000;
    if (width>max_cells || height>max_cells || width*height>max_cells) {
      ++output.grid_limit_rejections;
      ++output.clusters.candidates;
      continue;
    }
    cv::Mat observed(static_cast<int>(height),static_cast<int>(width),CV_8UC1,cv::Scalar(0));
    for (auto & p : pixels) {
      p.x=p.x-min_x+padding; p.y=p.y-min_y+padding;
      observed.at<std::uint8_t>(p.y,p.x)=1;
    }
    cv::Mat cleaned=observed.clone();
    auto morphology = [&](int operation, int radius) {
        if (radius==0) {return;}
        const auto kernel=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(2*radius+1,2*radius+1));
        cv::morphologyEx(cleaned,cleaned,operation,kernel,cv::Point(-1,-1),1,
          cv::BORDER_CONSTANT,cv::Scalar(0));
      };
    morphology(cv::MORPH_CLOSE,o.closing_radius_cells);
    morphology(cv::MORPH_OPEN,o.opening_radius_cells);
    cv::Mat labels;
    const int label_count=cv::connectedComponents(cleaned,labels,8,CV_32S);
    struct Component
    {
      std::vector<cv::Point2f> corners;
      std::size_t occupied{0}, points{0}, support{0};
      std::uint32_t output_id{0};
    };
    std::vector<Component> components(static_cast<std::size_t>(label_count));
    // Shape evidence uses original occupied cells that survived cleanup. Closing
    // can connect small gaps, but generated pixels never count as observed area.
    for (int y=0;y<observed.rows;++y) {
      for (int x=0;x<observed.cols;++x) {
        if (!observed.at<std::uint8_t>(y,x)) {continue;}
        const int label=labels.at<int>(y,x);
        if (label==0) {continue;}
        auto & component=components[label];
        ++component.occupied;
        const float fx=static_cast<float>(x), fy=static_cast<float>(y);
        for (const auto & delta : std::array<cv::Point2f,4>{{{0,0},{1,0},{1,1},{0,1}}}) {
          component.corners.emplace_back(fx+delta.x,fy+delta.y);
        }
      }
    }
    for (const auto & p : pixels) {
      const int label=labels.at<int>(p.y,p.x);
      if (label==0) {continue;}
      auto & component=components[label];
      ++component.points;
      if (input.points.xyz[p.index*3+2]>=c.support_height_m) {++component.support;}
    }
    for (int label=1;label<label_count;++label) {
      auto & component=components[label];
      ++output.clusters.candidates;
      if (component.occupied==0) {continue;}
      const auto box=cv::minAreaRect(component.corners);
      const double short_side=std::min(box.size.width,box.size.height);
      const double long_side=std::max(box.size.width,box.size.height);
      const double area=component.occupied*o.cell_size_m*o.cell_size_m;
      const double fill=component.occupied/(short_side*long_side);
      // Rotated bounds distinguish a diagonal thin line from a thick blob.
      if (short_side<=0 || area+1e-10<o.min_area_m2 ||
        short_side*o.cell_size_m+1e-8<o.min_thickness_m || fill+1e-8<o.min_fill_ratio ||
        long_side/short_side>o.max_aspect_ratio+1e-8 ||
        component.points<static_cast<std::size_t>(c.min_points) ||
        component.support<static_cast<std::size_t>(c.min_support_points) ||
        static_cast<double>(component.support)/component.points<c.min_support_ratio) {continue;}
      component.output_id=static_cast<std::uint32_t>(++output.clusters.accepted);
    }
    for (const auto & p : pixels) {
      output_labels[p.index]=components[labels.at<int>(p.y,p.x)].output_id;
    }
  }
  for (std::size_t i=0;i<output_labels.size();++i) {
    if (output_labels[i]==0) {continue;}
    output.clusters.points.xyz.insert(output.clusters.points.xyz.end(),
      input.points.xyz.begin()+i*3,input.points.xyz.begin()+i*3+3);
    output.clusters.ids.push_back(output_labels[i]);
  }
  output.clusters.points.width=static_cast<std::uint32_t>(output.clusters.ids.size());
  output.clusters.points.valid_points=output.clusters.ids.size();
  return output;
}
}  // namespace point_cloud
