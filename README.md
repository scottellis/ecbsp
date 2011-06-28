  ecbsp
=======

Linux OMAP3 driver using the McBSP controller to operate some motors over a
SPI interface.

Overo expansion board 

	Pin	Signal
	27	mcbsp_clkx
	28	mcbsp_dr
	29	mcbsp_fsx
	30	mcbsp_dx


  u-boot
-------

The driver requires some pin mux changes. The recipe and mux patch are in the
patches directory. 

Copy the patch to 

	${OVEROTOP}/org.openembedded.dev/recipes/u-boot/u-boot-omap3-git/

and replace the recipe here

	${OVEROTOP}/org.openembedded.dev/recipes/u-boot/u-boot-omap3_git.bb


Then rebuild u-boot

	$ bitbake -c clean virtual/bootloader
	$ bitbake virtual/bootloader


There is also a kernel patch and modified 2.6.36 recipe if you want a little
more debug from the plat/mcbsp.c driver. It's not necessary.


  build
-------

There is file to source [overo-source-me.txt] that should properly set up
your paths. You may have to modify the OETMP variable if you are using a 
non-standard location. 

After that, the makefile should work.


  test
-------

So we have basic functionality now. Once you load the driver you should immediately
start seeing activity on the CLKX line and FSX should be high. The clock should
be running at ~1MHz. You can specify three module params

	num_motors=(1-1024)
	delay_us=(1-1000)
	use_hrtimer=(0 or 1)

You can only changes these parameters on driver load for now. This will change.

If you then invoke the write function, you should see num_motor pulses of the 
FSX line. For each 32-bit transfer the FSX line is held low representing the 
SPI CS signal. Between each 32-bit transfer the FSX line should go high for 2 
clock pulses. This is configurable. I just hard-coded 2.

Here's an example session, fooling around with delays. Need a scope to watch
what's happening. 

	root@tide:~# insmod ecbsp.ko delay_us=46 num_motors=48
	[ 7650.548400] use_hrtimer = 0  delay_us = 46
	root@tide:~# echo write > /dev/ecbsp 

	root@tide:~# rmmod ecbsp.ko 

	root@tide:~# insmod ecbsp.ko delay_us=56 num_motors=48
	[ 7863.298736] use_hrtimer = 0  delay_us = 56
	root@tide:~# echo write > /dev/ecbsp 

	root@tide:~# rmmod ecbsp.ko 

	root@tide:~# insmod ecbsp.ko delay_us=86 num_motors=48
	[ 8498.462493] use_hrtimer = 0  delay_us = 86
	root@tide:~# echo write > /dev/ecbsp 

Right now the driver is hard-coded to send 100 blocks of num_motors with the 
delay you specify in between each. The delay you give has to compensate for
the data transfer since we starting the new block delay from the DMA callback
not the McBSP transfer complete callback. This may change. Still working out
what we want to do for the actual project.

The hard-coded clock speed is set at ~41.5MHz. Empirically, an estimate ~750ns
for each 32-bit transfer is good for compensating the delay at this speed.

48 * 750ns = 36us, so that's why you see delay_us = 46. That turns into a delay
between blocks of 10 us.

Hey! It's a work in progress ;-)





