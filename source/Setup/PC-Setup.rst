.. Steps for setting up the lab workspace on a personal computer
   10/09/24
   Abhishekh Reddy

Using a Personal Computer
=========================

**Skip this page if you're working on a lab computer.**

If you wish to work on the exercises outside the lab, this page helps you to set up the lab
workspace on a personal computer.

Requirements
^^^^^^^^^^^^

Operating System
----------------

Any recent release of Ubuntu (20.04 Focal and later) is recommended.

Other Linux distributions, particularly Ubuntu-based ones like Linux Mint or Pop!_OS, can also be
used. However, specific instructions or support for these or other distributions are beyond the
scope of this course.

Softwares
---------

- Git - ``apt install git``
- Docker - `documentation <Docker Install Documentation_>`_
- Visual Studio Code - `website <VSCode Download Link_>`_
    - Dev Containers Extension - `website <Dev Containers Extension Link_>`_
    - Docker Extension - `website <Docker Extension Link_>`_
- Additional packages for using GPU - `next section <GPU Acceleration (Optional)_>`_

.. note:: Make sure to also follow the `post-installation steps <Docker Post Installation Steps_>`_
          for Docker.

GPU Acceleration (Optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The base configuration, ``467-base``, utilizes either the integrated GPU or no GPU at all, depending
on the processor for graphics applications (e.g., Gazebo and RViz). However, it is compatible
with a wide range of computers.

Simulation performance can be significantly enhanced by using a discrete GPU if your computer
supports one.

NVIDIA GPUs
-----------

Install NVIDIA Container Toolkit on the host operating system by following the steps from
`documentation <NVIDIA Container Toolkit Link_>`_.

Then use the ``467-nvidia-gpu`` configuration when starting the Dev Container.

AMD GPUs
--------

Install AMD ROCm software on the host operating system by following the steps from
`documentation <AMD ROCm Software Link_>`_.

Then use the ``467-amd-gpu`` configuration when starting the Dev Container.

Starting the Container
^^^^^^^^^^^^^^^^^^^^^^

They're similar to the steps shown in the :ref:`this page <opening the workspace in visual studio code>`.
Use any of the ``467-<gpu-config>`` configurations when reopening the workspace folder in container.
The quickest way to get started is to use the ``467-base`` configuration.

.. LINK REFERENCES -------------------------------------------------------------

.. _Docker Install Documentation: https://docs.docker.com/engine/install/ubuntu/
.. _Docker Post Installation Steps: https://docs.docker.com/engine/install/linux-postinstall/
.. _VSCode Download Link: https://code.visualstudio.com/download
.. _Dev Containers Extension Link: https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers
.. _Docker Extension Link: https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker
.. _NVIDIA Container Toolkit Link: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
.. _AMD ROCm Software Link: https://rocm.docs.amd.com/projects/install-on-linux/en/latest/tutorial/quick-start.html#rocm-amdgpu-quick
