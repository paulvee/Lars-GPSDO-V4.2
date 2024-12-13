# Lars-GPSDO-V4.2
This is a significant improvement to the first version that can be found in a different repository.
There is also a companion board that interfaces to the outside world with insulated signals. This is a separate project called GPSDO V4 Interface.

The BOM's are of limited value because I do not specify the actual mfg partnumbers that can be ordered. I merely specify the value and the footprint.

The Arduino Nano sketch has a number of improvements, most notably the support for obtaining the Qerr value from the NEO, the automatic TC functionality, support for a room temperature sensor and a real 16-bit DAC.

A lot of detail can be found on my Blog here: https://www.paulvdiyblogs.net/2023/06/gpsdo-version-4.html
Be sure to visit that regularly to see if there are any updates.

The boards can be ordered through PCBWay, or the Gerbers can be downloaded there. I have created a Shared Project. Search for DIY GPSDO. 
PCBway had some questions about the parts they used for a customer that wanted a full assembly. I added comments to a document that they used to ask for my input.
The Q&A documents is called "QA for BOM 10MHz GPSDO V4.2.pdf"
I also included the Centroid files for both sides, they have the .pos suffix

It seems that JLPCB is not happy with the standard Gerber information, so I produced a special ZIP file that was produced with their "production" plug-in for KiCad. It should have all the information in it they want. Note that I also added the seperate files into the Gerber ZIP file so it can be uploaded in one go.
