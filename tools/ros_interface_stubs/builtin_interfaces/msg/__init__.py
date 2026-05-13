class Time:
    @classmethod
    def get_fields_and_field_types(cls):
        return {"sec": "int32", "nanosec": "uint32"}
