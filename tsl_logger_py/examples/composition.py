# Copyright 2024 Simon Sagmeister
from tsl_logger_py import ValueLogger, LoggerComposer

if __name__ == "__main__":
    logger1 = ValueLogger()
    logger2 = ValueLogger()
    logger3 = ValueLogger()

    logger1.log("a", 1.0)
    logger2.log("b", 1.0)
    logger3.log("c", 1.0)

    composer = LoggerComposer([logger1, logger2])

    composer.register_logger(logger3, "logger3/")

    print("The composer logged the following values:")
    for signal in composer.get_data():
        print("- " + signal)
