# SDN-based K-Means network Clustering of a platoon of 100 vehicles in NS-3 

To run the code, follow the instructions [here](http://www.lrc.ic.unicamp.br/ofswitch13/ofswitch13.pdf) to install _ns-3_ with the _ofswitch13_ module to enable OpenFlow support. YOu must build and configure first as mentioned using `./ns3 configure` and `./ns3 build`. You will then need to move the contents of this repostitory to the ns-3 `scrtach` directory

```
mv * /path/to/ns-3/scratch/
```

Now you need to build ns3 again to for the new application

```
./ns3 build
```

Now, you are ready to run the application

```
./ns3 run vehicular_network
```

This will generate an animation file that you can run with NetAnim. First you need to install the NetAnim executable and run using 

```
./NetAnim 
```
Once you open, you should load the animated *.xml file generated from the application. 


