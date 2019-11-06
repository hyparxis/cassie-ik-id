.. toctree::
   :maxdepth: 2
   :caption: Contents:



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

Inverse dynamics with Holonomic Constraints
===========================================

The dynamics of a rigid body tree with :math:`n` unconstrained degrees of freedom
and :math:`m` holonomic constraints can be expressed as:

.. math::
    \begin{gather}
    M \ddot{q} + h = Bu + J^T f + G^T \lambda \\
    g(q) = 0
    \end{gather}

where :math:`M` is the inertia matrix, :math:`h` is the vector of bias forces
including gravity and velocity-product terms, :math:`B` is the actuator
selection matrix, :math:`J^T` is the jacobian transpose mapping from cartesian forces 
to generalized forces, :math:`g : \mathbb{R}^n \mapsto \mathbb{R}^m` is the
vector-valued holonomic constraint function, :math:`G` is the jacobian of :math:`g` and
:math:`\lambda` is the vector of "virtual" forces/lagrange multipliers used to satisfy 
the constraint.

We can eliminate the constraint forces and yield a system in minimal 
coordinates by projecting the dynamics into the null-space of :math:`G`.
One way of accomplishing this is by choosing indices 
:math:`ind \in \mathbb{Z}^{n-m}` and :math:`dep \in \mathbb{Z}^m`
such that

.. math::
    G \dot{q} = 
    \left[
    \begin{array}{cc} 
    G_{ind} &
    G_{dep}
    \end{array}
    \right]
    \left[
    \begin{array}{c} 
    \dot{q}_{ind} \\
    \dot{q}_{dep}
    \end{array}
    \right] = 0

Then the matrix

.. math::
    \gamma = 
    \left[
    \begin{array}{c} 
    I \\
    -G_{dep}^{-1} G_{ind}
    \end{array}
    \right]

projects vectors into the null-space of g with the additional properties 
that 

.. math::
    \begin{gather}
    \gamma \dot{q}_{ind} = \dot{q} \\
    \gamma^T \dot{q} = \dot{q}_{ind}
    \end{gather}

So the constrained dynamics can be re-written

.. math::
    \gamma^T \left( M \ddot{q} + h \right) = \gamma^T \left( Bu + J^T f \right)

If :math:`u \in \mathbb{R}^{n-m}` (i.e. the system is fully actuated) then a 
natural choice of :math:`ind` is the set of actuated indices, which allows
for the feedback cancellation law

.. math::
    u = \gamma^T B^{-1} \left[ M \ddot{q} + C \dot{q} - J^T f \right]  


Rank Deficient Constraint Jacobian
==================================
In many cases the constraint jacobian might be rank deficient. This can arise
due to redundant constraints (e.g. arising from contact) or under-parameterized 
constraint equations (e.g. expressing a one or two dimensional constraint in
a three dimensional global reference frame). Notice that the projection matrix
:math:`\gamma` is only related to :math:`G` by its null space, so we can replace
:math:`G` with a row-equivalent (and therefore null space equivalent) matrix
:math:`R` that has full rank. We can find such a matrix using an orthogonal 
decomposition, i.e.

.. math::
    G = Q 
    \left[
    \begin{array}{c}
    R \\
    0
    \end{array}
    \right]
