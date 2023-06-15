import logging


class Logger:
    """
    Initialize logger for LS automation framwework
    """

    default_level = "INFO"
    LOGGERS_ERROR_ONLY = ["faker", "asyncio"]

    default_formatter = logging.Formatter("%(asctime)s - {%(filename)s:%(lineno)d} - %(name)-15s - %(levelname)-8s - %(message)s")

    def __init__(self) -> None:
        loglevel = Logger.default_level

        numeric_level = getattr(logging, loglevel.upper(), None)
        if not isinstance(numeric_level, int):
            raise ValueError(f"Invalid log level: {loglevel}")
        self.loglevel = numeric_level

    def init_logger(self):
        """
        Initialize standard loggers.

        Use it in case you need to disable any standart logger
        """
        # faker logger spams to logs.
        for log in self.LOGGERS_ERROR_ONLY:
            logging.getLogger(log).setLevel(logging.ERROR)

        return self

    def get_logger(self, logger_name):
        """
        Return logger for a module.
        Add handlers here if you need to write not only to console
        """
        rootLogger = logging.getLogger(logger_name)

        consoleHandler = logging.StreamHandler()
        consoleHandler.setFormatter(Logger.default_formatter)

        rootLogger.addHandler(consoleHandler)
        rootLogger.setLevel(level=self.loglevel)

        return rootLogger


# Init singleton object for all the modules
SDVLogger = Logger().init_logger()
