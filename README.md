# meanwell-enc-series-chargelog
Command line utility to program or monitor the Mean Well ENC-series of battery chargers.  
No programmer needed. Direct connection to the serial port of the charger (instructions given in the file header).  
Programming of the charging curve, i.e. programming of constant current, constant voltage, taper current, float voltage and temperature coefficient is supported.  
Monitoring logs voltage and current over time, shows a graph from time to time.  

Platform independent (Python).  
Can simply be used remotely in a terminal for example. No graphic interface needed. The graph is displayed using characters.  
Logs can be easily imported as spreadsheets. The format is comma-separated values.  
An interrupted monitoring can be continued. Values from the interrupted log are read in and added to the graph.  

Here you can see how the output and the log look like: sanity_check\mockup.txt and mockup.log.  

ENC-120-12 / ENC-180-12 / ENC-240-12 / ENC-360-12  
ENC-120-24 / ENC-180-24 / ENC-240-24 / ENC-360-24  
ENC-120-48 / ENC-180-48 / ENC-240-48 / ENC-360-48  