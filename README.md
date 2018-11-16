[![Build status](https://badge.buildkite.com/a424560339e282f68e230109b50decf4f7ed5b72afa93e1326.svg?branch=master)](https://buildkite.com/fawkesrobotics/fawkes-build)
[![Total alerts](https://img.shields.io/lgtm/alerts/g/fawkesrobotics/fawkes.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/fawkesrobotics/fawkes/alerts/)
[![Language grade: C/C++](https://img.shields.io/lgtm/grade/cpp/g/fawkesrobotics/fawkes.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/fawkesrobotics/fawkes/context:cpp)
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/b107df4d77aa46a386c81faf9c425544)](https://www.codacy.com/app/fawkesrobotics/fawkes)

# Fawkes Robot Software Framework
Fawkes is a component-based Software Framework for Robotic Real-Time Applications for various
Platforms and Domains.

It is developed and used for cognitive robotics real-time applications such as soccer, domestic
service, or industrial mobile robots. It supports fast information exchange and efficient
combination and coordination of different components to suit the needs of mobile robots operating in
uncertain environments.

Today, Fawkes consists of an efficient communication middleware with a hybrid blackboard and
messaging design, a flexible set of software libraries to make developing robotic functionalities
reasonably simple, and a coherent development and run-time environment with tools that make
integrating and running robotic applications achievable. But Fawkes also comprises a large set of
plugins, software components that implement typical robot functionality like self-localization, path
planning, perception, or behavior execution and monitoring. Several robot platforms like the
Robotino, Nao, or Roomba are supported out-of-the-box. Fawkes also integrates with other software
frameworks like ROS or Player and integrates third party software like MongoDB, Prometheus, CLIPS,
or OpenNI. By using ideas from aspect-oriented programming, and run-time requirement assertion by
the framework, it is easy to access these resources with only a few lines of code.
