#!/usr/bin/env python

import datetime
import requests


class GoPro:

    def __init__(self, ip_address, timeout=5):
        self.ip_addr = ip_address
        self._timeout = timeout

    def _request(self, path, param="", value="", _timeout=None, _isHTTPS=False, _context=None):

        if _timeout == None:
            _timeout = self._timeout

        if param != "" and value == "":
            uri = "%s%s/%s/%s" % ("https://" if _isHTTPS else "http://",
                                  self.ip_addr, path, param)
        elif param != "" and value != "":
            uri = "%s%s/%s/%s/%s" % ("https://" if _isHTTPS else "http://",
                                     self.ip_addr, path, param, value)
        elif param == "" and value == "":
            uri = "%s%s/%s" % ("https://" if _isHTTPS else "http://",
                               self.ip_addr, path)
        print('Sending request: {}'.format(uri))
        r = requests.get(url=uri, timeout=_timeout)
        data = r.json()

        print('Response: {}'.format(data))
        return data

    def gpControlCommand(self, param):
        """sends Parameter gpControl/command"""
        try:
            return self._request("gp/gpControl/command/" + param)
        except Exception:
            return ""

    def gpWebcam(self, param):
        """sends Parameter to gpWebcam"""
        try:
            return self._request("gp/gpWebcam/" + param)
        except Exception:
            return ""

    def isWebcam(self):
        return self.ip_addr.startswith("172") and self.ip_addr.endswith("51")

    def syncTime(self):
        """Sets time and date to computer"s time and date"""
        now = datetime.datetime.now()
        year = str(now.year)[-2:]
        datestr_year = format(int(year), "x")
        datestr_month = format(now.month, "x")
        datestr_day = format(now.day, "x")
        datestr_hour = format(now.hour, "x")
        datestr_min = format(now.minute, "x")
        datestr_sec = format(now.second, "x")
        datestr = str("%" + str(datestr_year)+"%"+str(datestr_month)+"%"+str(
            datestr_day)+"%"+str(datestr_hour)+"%"+str(datestr_min)+"%"+str(datestr_sec))
        return self.gpControlCommand("setup/date_time?p=" + datestr)

    def startWebcam(self, resolution="1080"):
        print('Starting webcam with resolution: {}'.format(resolution))
        return self.gpWebcam("START?res=" + resolution)

    def stopWebcam(self):
        return self.gpWebcam("STOP")

    def webcamFOV(self, fov="0"):
        return self.gpWebcam("SETTINGS?fov=" + fov)
