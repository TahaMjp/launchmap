class ParseContext:
    def __init__(self, visitor=None, field=None):
        self.visitor = visitor
        self.field = field

    def track_launch_arg_usage(self, arg_name):
        if self.visitor and self.field:
            self.visitor.track_launch_arg_usage(arg_name, self.field)