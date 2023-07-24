# Copyright 2023 FZI Forschungszentrum Informatik


class BehaviorTreeException(Exception):
    pass


class NodeConfigError(BehaviorTreeException):
    pass


class NodeStateError(BehaviorTreeException):
    pass


class TreeTopologyError(BehaviorTreeException):
    pass


class MissingParentError(BehaviorTreeException):
    pass


class MigrationException(BehaviorTreeException):
    pass


class AssignmentException(Exception):
    pass
