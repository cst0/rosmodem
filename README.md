# ROSModem: Simplify the integration of modems and wireless devices in ROS

This package aims to standardize and simplify the integration of various
(non-ros, non-wifi) communications strategies into a ROS ecosystem. This is
done by providing a common interface (the 'Modem' class) which can be extended.
The user the implements the `read` and `write` functions, and the `Modem` class
will handle pulling in ROS topics, sending them across your modem, and making
them available on the other end. While we're at it, we'll handle some
compression and serialization.
