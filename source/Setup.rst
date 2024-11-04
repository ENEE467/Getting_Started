.. Index for workspace setup
   10/09/24
   Abhishekh Reddy

Setting up the Workspace
=========================

Workspace folder for the lab exercises is hosted on this `GitHub repository <GitHub Repository_>`_
for reference.

For saving the work on GitHub (Optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you wish to save the work on GitHub, `create a new repository <Create Repository Template_>`_ on
your GitHub account using this repository as a template.

Clone the workspace
^^^^^^^^^^^^^^^^^^^

Start a bash terminal by pressing the ``CTRL + ALT + T`` keys. Follow the steps below and run the
commands in this window.

Download the workspace folder by cloning the git repository in the ``home/`` directory. Replace
the ``<workspace-name>`` field with a name identifiable to your group.

.. code-block:: bash

  git clone https://github.com/ENEE467/lab-workspace.git ~/<workspace-name>

Also make sure to use the URL of your repository instead if applicable.

.. _opening in vscode:

Opening the workspace in Visual Studio Code
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use ``Open Folder`` option from the ``File`` menu in VSCode or use the shell command.

.. code-block:: bash

   code ~/<workspace-name>

Then either click ``Reopen in Container`` from the pop-up at the bottom-right part of the window.

.. figure:: images/reopen-1.png
   :width: 450
   :align: center

   Opening in Dev Container through pop-up

Or click the remote button at the bottom-left corner of the window in case you missed the pop-up.

.. figure:: images/reopen-2.png
   :width: 450
   :align: center

   Opening in Dev Container using remote button

Always select the ``ral-lab`` configuration **when doing the exercises on a lab computer**.

.. figure:: images/config-select.png
   :width: 450
   :align: center

   Selecting the configuration on lab computer

The container should start and execute the setup script which primarily involves building the
packages.

Finally, source the terminal before use just for the first time.

.. code-block:: bash

   source ~/.bashrc

.. note::

   .. figure:: images/bash.png
      :width: 450
      :align: center

      VSCode Terminal

   After configuring the container, a terminal should automatically start at the bottom of the
   VSCode window. If it doesn't, press ``Ctrl + ``` or go to ``View > Terminal`` to open it.

   You will use this terminal to execute most of the commands in the following lab exercises.

Next Steps
^^^^^^^^^^

You can now get started with :doc:`Lab7` and :doc:`Lab8` exercises.

Optional Steps
^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 1
   :titlesonly:

   Setup/Attach-Shell
   Setup/PC-Setup

.. LINK REFERENCES ---------------------------------------------------------------------------------

.. _GitHub Repository: https://github.com/ENEE467/lab-workspace
.. _Create Repository Template: https://github.com/new?template_name=lab-workspace&template_owner=ENEE467
