from src.config.converters.base_converter import BaseConverter


class BoolConverter(BaseConverter):
    CLASS = bool

    @classmethod
    def convert(cls, val):
        if str(val).lower() in ["1", "true", "t"]:
            return True
        elif str(val).lower() in ["0", "false", "f"]:
            return False
        else:
            raise ValueError(f"Impossible to convert {val} to {cls.CLASS} type")
