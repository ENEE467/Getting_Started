.. Steps for building the Docker image
   01/09/24
   Abhishekh Reddy

Building the Docker Image
=========================

This repository has the required Docker image for the lab course that needs to
be built. Clone it in your favorite directory.

.. code-block:: bash

  git clone https://github.com/ENEE467/documentation.git lab-environment

Navigate to the cloned folder and build the Docker image

.. code-block:: bash

  cd lab-environment/docker

.. code-block:: bash

  DOCKER_BUILDKIT=1 docker build --build-arg USER=$USER \
             --build-arg UID=$(id -u) \
             --build-arg GID=$(id -g) \
             --build-arg PW=docker \
             -t ur3e_image \
             -f noetic_dockerfile.Dockerfile\
             .

Instructions to run this built image can be found in any of the Lab exercise
instructions.

=====================   ========================================================
Argument, Shorthand                           Description
=====================   ========================================================
``DOCKER_BUILDKIT=1``   Improvement in performance, storage management, feature
                        functionality, and security

``--tag``, ``-t``       Name and optionally a tag in the ``name:tag`` format

``--file``, ``-f``      Name of the Dockerfile (Default is ``PATH/Dockerfile``)

``.``                   Path to current directory relative to root
=====================   ========================================================
