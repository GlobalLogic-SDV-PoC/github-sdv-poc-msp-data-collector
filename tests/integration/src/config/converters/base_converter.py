class BaseConverter:
    CLASS = None

    @classmethod
    def convert(cls, val):
        if cls.CLASS is None:
            raise NotImplementedError(f"Converter {__class__.__name__} is not implemented for type {cls.CLASS}")

        try:
            converted_val = cls.CLASS(val)
        except Exception as e:
            raise ValueError(f"Impossible to convert {val} to {cls.CLASS} type. Error details: {e}")

        return converted_val
