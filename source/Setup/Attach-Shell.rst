.. Steps for attaching a host shell to the devcontainer
   10/10/24
   Abhishekh Reddy

Accessing the Dev Container from the Host Terminal
==================================================

In addition to the VSCode terminal, you can use a terminal in the host OS to access and execute
commands inside the container.

Press the ``CTRL + ALT + T`` keys to start a bash terminal.

Use the ``docker exec`` command to access the container.

.. code-block:: bash

    docker exec -it -u 467-terp <workspace-name> bash

Replace the ``<workspace-name>`` field with the name of your workspace folder. Container uses the
same name as the workspace folder.
