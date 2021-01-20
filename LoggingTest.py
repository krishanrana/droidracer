# Code taken from https://www.golinuxcloud.com/python-logging/

import logging
import sys
import logging.config

LogConfig = 1

# To be used with logging.conf file

""" [loggers]
keys=root,my_logger

[handlers]
keys=consoleHandler,fileHandler

[formatters]
keys=consoleFormatter,fileFormatter

[logger_root]
level=DEBUG
handlers=consoleHandler

[logger_my_logger]
level=DEBUG
handlers=consoleHandler,fileHandler
qualname=my_logger
propagate=0

[handler_consoleHandler]
class=StreamHandler
level=CRITICAL
formatter=consoleFormatter
args=(sys.stdout,)

[handler_fileHandler]
class=FileHandler
level=DEBUG
formatter=fileFormatter
args=('/tmp/debug.log', 'a')

[formatter_consoleFormatter]
format=%(levelname)-8s %(name)-12s %(message)s

[formatter_fileFormatter]
format=[%(asctime)s] %(levelname)-8s %(name)-12s %(message)s """
if LogConfig:
    # Define the logging.conf filePath
    logging.config.fileConfig('logging.conf', disable_existing_loggers=False)

    # Define your own logger name
    logger = logging.getLogger("droidlogger")

    # Write messages with all different types of levels
    logger.debug('debug')
    logger.info('info')
    logger.warning('warning')
    logger.error('error')
    logger.critical('critical')


# Code to place in script
else:
    # Log file location
    logfile = 'debug.log'


    # Define your own logger name
    logger = logging.getLogger("my_logger")
    # Set default logging level to DEBUG
    logger.setLevel(logging.DEBUG)

    # create console handler
    print_format = logging.Formatter('%(levelname)-8s %(name)-12s %(message)s')
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.WARNING)
    console_handler.setFormatter(print_format)

    # create log file handler
    # and define a custom log format, set its log level to DEBUG
    log_format = logging.Formatter('[%(asctime)s] %(levelname)-8s %(name)-12s %(message)s')
    file_handler = logging.FileHandler(logfile)
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(log_format)

    #Add handlers to the logger
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)

    # Write messages with all different types of levels
    logger.debug('debug')
    logger.info('info')
    logger.warning('warning')
    logger.error('error')
    logger.critical('critical')