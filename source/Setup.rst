.. Index for environment setup
   01/09/24
   Abhishekh Reddy

Setting Up the Environment
==========================

All the lab exercises involving ROS use Docker containers for setting up the
work environment. This greatly simplifies the setup process since the right
Ubuntu distribution, installation of ROS and dependent packages all come bundled
in a single Docker image. Running this image as a container decouples the
working environment from the host operating system.

Although Docker can be installed and used on all the mainstream operating
systems (Linux, Windows and macOS), we strongly recommend using Linux since the
Lab computers use Linux (Specifically Ubuntu) and ROS provides `Tier 1 support
<ROS Support Link_>`_ for Ubuntu which guarantees maximum stability and
functionality.

.. note::
   Skip this part on lab computers since it's already done for you. Follow these
   instructions if you wish to use a personal computer for the lab exercises.

Go through the following steps to set up the environment for lab exercises.

.. toctree::
   :maxdepth: 1
   :titlesonly:

   Setup/Installing-Docker
   Setup/Additional-Setup
   Setup/Build-Docker-Image

.. LINK REFERENCES -------------------------------------------------------------
.. _ROS Support Link: https://ros.org/reps/rep-2000.html#humble-hawksbill-may-2022-may-2027
