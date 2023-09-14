# ROS UTILS

## Overview

The `ros_utils` package offers a C++ library designed to simplify ROS (Robot Operating System) development by providing utility functions for common ROS tasks. It includes features like parameter retrieval and logging level configuration, along with a versatile template class called `RecentMessageSubscriber`. Overall, the package is aimed at enhancing the efficiency and ease of ROS development.

### Features

- `GetParams`: A template function to retrieve a parameter's value from the ROS parameter server, with error handling.
- `ConfigureLogLevel`: Function to set the ROS log level based on a command-line argument.
- `RecentMessageSubscriber`: A template class for subscribing to a ROS topic and obtaining the most recent message of a specified type.
- `Custom ROS exception classes`: Provides custom exception classes tailored for ROS-related errors.
- `Detailed documentation using Doxygen`: The package includes Doxygen documentation to help users understand and utilize the library effectively.

## RecentMessageSubscriber

The `RecentMessageSubscriber` is a versatile template class designed to simplify message handling within the ROS (Robot Operating System) framework. It allows you to subscribe to a ROS topic and retrieve the most recent message received on that topic, making it useful for various tasks, including message synchronization between different topics. The class's key components include:

- **Constructor**: Initializes the subscriber and sets up necessary callbacks.
- **Subscriber Callback**: A function that handles incoming messages and updates the stored message and timestamp with the most recent data.
- **GetRecentMessage**: Retrieves the most recent received message and checks if it is within a specified timeout. If the message is stale or an invalid timeout value is provided, it returns `nullptr`.

### Key Features

- **Subscription and Retrieval**: The class manages the subscription to a ROS topic and stores the most recent message received, providing a convenient way to access this message.

- **Template Flexibility**: Templated to work with different message types, allowing you to use it with various ROS message formats.

- **Timeout Handling**: You can specify a timeout value to ensure that the retrieved message is up-to-date. If the message is not available or if the specified timeout is exceeded, the message is considered stale.

### Message Synchronization

One common use case for the `RecentMessageSubscriber` is message synchronization. Suppose you have two different ROS topics published at different rates, and you want to access the most recent messages from both topics efficiently. By creating two `RecentMessageSubscriber` instances, each subscribed to a different topic, you can effectively synchronize access to the latest data from each topic.

Here's how you can use `RecentMessageSubscriber` for message synchronization:

1. Create two instances of `RecentMessageSubscriber`, each subscribing to a different ROS topic.

2. Each instance will maintain its copy of the most recent message received from its respective topic.

3. Periodically check both instances to access the latest messages, ensuring synchronized access to data from different topics.

This approach allows you to work with the most up-to-date information from each topic, preventing the processing of stale data.

Keep in mind that this synchronization is specific to accessing recent messages from individual topics and does not address more complex message synchronization scenarios involving alignment based on timestamps or coordination across multiple topics.

For advanced message synchronization tasks requiring coordination between messages from different topics, consider additional logic and libraries like ROS's `message_filters`.

For more details on using the `RecentMessageSubscriber` class and examples, refer to the class's source code and documentation.

## Dependencies

- **Doxygen**: Used to generate helpful documentation about the library.

  - For information on how to get started with Doxygen, refer to the [documentation](./docs/README.md).
  - Full Doxygen documentation can be found at the official website [https://doxygen.nl/](https://doxygen.nl/).

## Author & Maintainer

Julian Rendon (julianrendon514@gmail.com)
