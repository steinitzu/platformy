import logging
import logging.handlers

LOG_LEVEL = logging.WARNING
def init_log():
    log = logging.getLogger('Ballmonster')
    hdlr = logging.StreamHandler()
    frmter = logging.Formatter(
        '%(levelname)s:%(asctime)s:%(msecs)d:%(module)s-line:%(lineno)d:%(message)s')
    hdlr.setFormatter(frmter)
    fhdlr = logging.handlers.RotatingFileHandler('programlog.log',
                                                 maxBytes=2000000,
                                                 backupCount=3)
    fhdlr.setFormatter(frmter)
    log.addHandler(hdlr)
    log.addHandler(fhdlr)
    log.setLevel(LOG_LEVEL)
    return log
