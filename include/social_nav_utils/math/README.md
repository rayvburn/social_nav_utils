# math

Custom classes for 2D linear algebra with API similar to `Eigen`.
`Eigen` library carried a too high computational burden for the robot online application, thus simple classes wrapping plain arrays were written.
Performance has increased 10 times thanks to these simple classes with the hard-coded number of stored elements.
