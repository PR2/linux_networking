#! /usr/bin/env python

import logging
import logging.handlers
import os
from logging_config import *
import fcntl
import time
import traceback
import signal
import select
from twisted.internet import protocol

class CommandWithOutput(protocol.ProcessProtocol):
    def __init__(self, args, name):
        self.restart_delay = 0.2
        self.logger = logging.getLogger(name)
        self.console_logger = logging.getLogger('console.%s'%name)
        logger_handler = logging.handlers.TimedRotatingFileHandler(os.path.join(logdir,'%s.log'%name), when='midnight', backupCount=logfilecount)
        logger_handler.setFormatter(file_formatter)
        self.logger.addHandler(logger_handler)
        self.console_logger.addHandler(logger_handler)
        self.logger.setLevel(logging.DEBUG)
        self.console_logger.setLevel(logging.DEBUG)
        logger_handler.setLevel(logging.DEBUG)
        self.proc_args = args
        self.outline = ""
        self.errline = ""
        self.shutting_down = False
        self.start_proc()
                            
    def errReceived(self, data):
        self.errline = self.dataReceived(self.errline, data)

    def outReceived(self, data):
        self.outline = self.dataReceived(self.outline, data)

    def data_received(self, curline, data):
        curline += data
        while True:
            splitpos = curline.find('\n')
            if splitpos == -1:
                break
            self.got_line(curline[splitpos])
            curline = curline[splitpos+1:]
        return curline

    def processEnded(self, status_object):
        if self.outline: 
            self.got_line(self.outline)
        if self.errline:
            self.got_line(self.errline)
        if self.shutting_down:
            return
        self.console_logger.info("Process died, restarting: %s"%(" ".join(self.proc_args)))
        self.start_proc()
    
    def start_proc(self):
        try:
            self.proc = reactor.spawnProcess(self, self.args[0], self.args, None)
            self.child_restart()
        except OSError:
            self.console_logger.fatal("Error trying to run: %s"%(" ".join(self.proc_args)))

    def child_restart(self):
        pass # Can be overridden by derived classes.

    def _got_line(self, line):
        self.logger.info(line)
        try:
            self.got_line(line)
        except Exception, e:
            self.console_logger.fatal("Caught exception in CommandWithOutput.run: %s"%str(e))
            raise # FIXME Remove this?

    def shutdown():
        self.shutting_down = True
        self.proc.signalProcess("INT")

if __name__ == "__main__":
    import unittest
    from async_helpers import unittest_with_reactor, async_test

    class CommandWithOutputTest(unittest.TestCase):
        @async_test
        def basic_test():
            class Tst(CommandWithOutput):
                def __init__():
                    CommandWithOutput.__init__(['ls', '/'], "test")

                def got_line():
                    self.shutdown()

    def run_ros_tests():
        rostest.unitrun('multi_interface_roam', 'command_with_output', CommandWithOutputTest)
    
    unittest_with_reactor(run_ros_tests)
