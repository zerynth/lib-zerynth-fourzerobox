Firmware Over The Air Update
============================

Connect your 4ZeroBox to ZDM and start updating the firmware seamlessly.

In this example, a FOTA callback function is defined, which is called during the FOTA update steps.
The FOTA callback allows you to accept or refuse a FOTA from your devices using the return value.
If the callback returns True the device will accept the FOTA update requests, if the callback return False
the device will refuse it.

Try to edit the function e do your tests using ZDM FOTA commands.