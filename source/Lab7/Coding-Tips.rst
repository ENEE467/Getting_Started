.. Tips for performing math using Eigen
   10/31/24
   Abhishekh Reddy

Tips for implementing the math
==============================

The code uses the `Eigen <Eigen Main Page_>`_ library for performing matrix and vector operations
necessary for calculating the vectors and covariance matrix. Refer to the
`quick reference guide <Quick Reference Guide_>`_ to find methods for various operations.

Operations on the pose and error vector
---------------------------------------

End-effector poses and measurement errors in code are stored as a vector with 7 components. The first
three components store XYZ positions and the remaining four store the four components of the
orientation quaternion.

.. math::

    \begin{bmatrix} x \\ y \\ z \\ q_x \\ q_y \\ q_z \\ q_w \end{bmatrix}

.. code-block:: C++

    double x, y, z, q_x, q_y, q_z, q_w;

    Eigen::Vector<double, 7> vector {x, y, z, q_x, q_y, q_z, q_w};

Vector math like additions, subtractions, multiplication/division with scalar values can be done
directly using normal C++ operators like ``+``, ``*``, ``/``.

The library supports element-wise operations on ``Eigen::Array`` objects but not on ``Eigen::Vector``
types. To perform element-wise operations on ``Eigen::Vector`` objects, you can use the ``array()``
method to obtain an ``Eigen::Array`` wrapper to apply operations on. The entire list of available
operations can be found `here <Coefficient Wise Operators_>`_.

For example, performing an element-wise square operation on a vector using the ``square()`` method
looks like this:

.. math::

    \begin{bmatrix} x^2 \\ y^2 \\ z^2 \\ {q_x}^2 \\ {q_y}^2 \\ {q_z}^2 \\ {q_w}^2 \end{bmatrix}

.. code-block:: C++

    auto vector_squared = Eigen::Vector<double, 7>(vector.array().square());

The returned value from ``square()`` method is not an ``Eigen::Vector`` object. It either needs
to be casted back to that type or used as a parameter for initializing a new object of that type.

For finding the transpose of the vector, simply use the ``transpose()`` method.

.. math::

    \begin{bmatrix} x \\ y \\ z \\ q_x \\ q_y \\ q_z \\ q_w \end{bmatrix}^T

.. code-block:: C++

    auto vector_transpose = vector.transpose();

Using the list of errors from measurements
------------------------------------------

When capturing the measurements for verification, the errors between the estimated end-effector poses
(measured by the camera after calibration) and the reference poses (from robot's forward kinematics)
are already calculated for you and are stored in a list as an ``std::vector`` object. The size of this
vector is the number of measurements taken.

.. math::

    errors = \{ e_1, e_2, e_3, \ ... \ e_n \}

.. code-block:: C++

    std::vector<Eigen::Vector<double, 7>> error_vecs;

    auto number_of_measurements = error_vecs.size();

A single error vector when indexed from this list would be:

.. math::

    e_i = \begin{bmatrix} x_i \\ y_i \\ z_i \\ {q_x}_i \\ {q_y}_i \\ {q_z}_i \\ {q_w}_i \end{bmatrix}

.. code-block:: C++

    auto e_i = error_vecs.at(i);

For accessing all the error vectors from the list, a simple range-based ``for`` loop will suffice.

.. code-block:: C++

    for (const auto& e_i: error_vecs) {

        // Do something with e_i

    }

Formulas for reference
----------------------

Sample Mean
^^^^^^^^^^^

.. math::

    \bar{e} = \frac{1}{n} \sum_{i=1}^{n} e_i

:math:`n` is the number of measurements.

Sample Covariance
^^^^^^^^^^^^^^^^^

.. math::

    S = \frac{1}{n - 1} \sum_{i=1}^{n} (e_i - \bar{e}) (e_i - \bar{e})^T

:math:`e_i - \bar{e}` is the deviation of an error from sample mean error.

Sum of Squared Errors
^^^^^^^^^^^^^^^^^^^^^

.. math::

    SSE = \sum_{i=1}^{n}
        \begin{bmatrix}
            x_i^2 \\
            y_i^2 \\
            z_i^2 \\
            {q_x}_i^2 \\
            {q_y}_i^2 \\
            {q_z}_i^2 \\
            {q_w}_i^2
        \end{bmatrix}

The square operation on the error vector :math:`e_i` is element-wise.

.. LINK REFERENCES ---------------------------------------------------------------------------------

.. _Eigen Main Page: https://eigen.tuxfamily.org/index.php?title=Main_Page
.. _Quick Reference Guide: https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
.. _Coefficient Wise Operators: https://eigen.tuxfamily.org/dox/group__QuickRefPage.html#:~:text=top-,Coefficient%2Dwise%20%26%20Array%20operators,-In%20addition%20to

