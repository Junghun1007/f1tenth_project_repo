"""Launch lifecycle helpers, imported without starting ROS or vehicle processes."""


class ShutdownOnce:
    """Share across critical children so cascading exits request shutdown once."""
    def __init__(self):
        self.requested = False

    def __call__(self, event, context):
        if context.is_shutdown or self.requested:
            return []
        self.requested = True
        from launch.actions import EmitEvent
        from launch.events import Shutdown
        return [EmitEvent(event=Shutdown(reason='Autotune critical process exited'))]
