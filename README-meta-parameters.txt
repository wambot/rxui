    Meta-parameters

rxui allows you to provide meta-information about parameters. This is useful
when using rxui to dynamically edit parameters.

To use this, simply create another parameter with '__meta' postfixed to its
name. For example:

    /some/parameter         <- a parameter
    /some/parameter__meta   <- the parameter's meta_information

This meta information is a table containing at least the key 'type' which
specifies what type of information the parameter should contain. The possible
values of 'type' are "int", "float", "bool" and "string". This table may also
contain other key-values pairs depending on the type:

For ['type'] == "int" or "float"
    min_value:  The minimum possible value of this parameter
    max_value:  The maximum possible value of this parameter
    increment:  The gap between reasonable values of the parameter. How much
                should the value change when the user moves the scroll bar in
                rxui by a single step.
    default:    The default value.
    scale:      Either "linear" or "logarithmic". How a scroll bar should
                alter this value. "linear" is the default if this entry is
                omitted.

For ['type'] == "bool":
    default:    The default value.

For ['type'] == "string":
    defines:    What the string defines. Currently the only possible value is
                "topic" which implies that the parameter must contain the name
                of a topic.
    topic_type: If ['defines'] == "topic" then this parameter contains the
                type of message that the topic is expected to carry. For
                example "sensor_msgs/Image".



  **  Example  **

A node creates two parameters. One contains a probability, the other contains
a string that refers to a bool-carrying topic. This is what the parameter tree
might look like:

    /node/probability                   : 0.034
    /node/probability__meta/type        : "float"
    /node/probability__meta/min_value   : 0
    /node/probability__meta/max_value   : 1
    /node/probability__meta/increment   : 0.001
    /node/probability__meta/default     : 0.5
    /node/probability__meta/scale       : "linear"
    /node/bool_src                      : "/some/topic"
    /node/bool_src__meta/type           : "string"
    /node/bool_src__meta/defines        : "topic"
    /node/bool_src__meta/               : "std_msgs/Bool"
