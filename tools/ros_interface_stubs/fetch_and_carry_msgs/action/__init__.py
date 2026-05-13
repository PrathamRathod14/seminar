class _Fields:
    @classmethod
    def get_fields_and_field_types(cls):
        return dict(cls._fields)


class Navigate:
    class Goal(_Fields):
        _fields = {"loc_id": "int32"}

    class Result(_Fields):
        _fields = {}

    class Feedback(_Fields):
        _fields = {}


class Pick:
    class Goal(_Fields):
        _fields = {"object_id": "int32"}

    class Result(_Fields):
        _fields = {}

    class Feedback(_Fields):
        _fields = {}


class Place:
    class Goal(_Fields):
        _fields = {}

    class Result(_Fields):
        _fields = {}

    class Feedback(_Fields):
        _fields = {}
