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

Math operations like additions, subtractions, multiplication/division with scalar values can be done
directly using normal C++ operators like ``+``, ``*``, ``/``.

For squaring the vector elements, access the array object using ``array()`` method and then use its
coefficient-wise ``square()`` method.

.. math::

    \begin{bmatrix} x^2 \\ y^2 \\ z^2 \\ {q_x}^2 \\ {q_y}^2 \\ {q_z}^2 \\ {q_w}^2 \end{bmatrix}

.. code-block:: C++

    auto vector_squared = Eigen::Vector<double, 7>(vector.array().square());

.. note::

    The returned value from ``square()`` method is not an ``Eigen::Vector`` object. It either needs
    to be casted to that type or used as a parameter for initializing a new object of that type.

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

    e_i = \begin{bmatrix} x_i \\ y_i \\ z_i \\ q_{xi} \\ q_{yi} \\ q_{zi} \\ q_{wi} \end{bmatrix}

.. code-block:: C++

    auto e_i = error_vecs.at(i);

For accessing all the error vectors from the list, a simple range-based ``for`` loop will suffice.

.. code-block:: C++

    for (const auto& e_i: error_vecs) {

        // Do something with e_i

    }

.. LINK REFERENCES ---------------------------------------------------------------------------------

.. _Eigen Main Page: https://eigen.tuxfamily.org/index.php?title=Main_Page
.. _Quick Reference Guide: https://eigen.tuxfamily.org/dox/group__QuickRefPage.html

