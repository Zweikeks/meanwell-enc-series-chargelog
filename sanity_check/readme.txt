No serial port, no charger needed.

Sanity Check 1
- set mockup = True at the beginning of the script
- start the script on the command line:
  chargelog.py mon com3 mockup.log -ri 60 -gi 1800 > mockup.txt
  (needs about 10 seconds to complete)
- compare the output with mockup.log and mockup.txt here in the directory

Sanity Check 2
- uncomment gtest() and sys.exit(0) in the main section at the end of the script
- start the script on the command line:
  chargelog.py > gtest.txt
- compare the output with gtest.txt here in the directory

This is a fairly brief test. The programming part and the part reading in old logs is not checked at all.
