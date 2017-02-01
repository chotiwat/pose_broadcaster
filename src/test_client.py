#!/usr/bin/env python3

import time
import sys
from broadcast_client import BroadcastClient

def main():
  client1 = BroadcastClient(0)
  client2 = BroadcastClient(1)
  try:
    while True:
      print(1, client1.query())
      print(2, client2.query())
      time.sleep(0.1)
  except KeyboardInterrupt:
    client1.close()
    client2.close()
    sys.exit(0)


if __name__ == '__main__':
  main()