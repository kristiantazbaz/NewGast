#!/usr/bin/env python

from __future__ import print_function

import SimpleHTTPServer
import SocketServer
import os
import rospy

if __name__ == "__main__":
  rospy.init_node('webserver', anonymous=True)

  web_dir = os.path.join(os.path.dirname(__file__), '..', 'web')
  os.chdir(web_dir)
  handler = SimpleHTTPServer.SimpleHTTPRequestHandler
  httpd = SocketServer.TCPServer(('', 8080), handler)
  httpd.timeout = 0.5

  rospy.loginfo('Serving at 127.0.0.1:8080')
  rate = rospy.Rate(5)
  while not rospy.is_shutdown():
    httpd.handle_request()
    rate.sleep()
  #httpd.serve_forever()

