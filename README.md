<p align="center"><img src="https://socialify.git.ci/AnmeetS/Ball-Balancer/image?description=1&amp;font=Bitter&amp;language=1&amp;name=1&amp;owner=1&amp;pattern=Circuit%20Board&amp;theme=Dark" alt="project-image"></p>

Hello and welcome to the GitHub repository of my Ball Balancing Robot! This project marks a significant milestone in my embdeded software engineering journey, combining my passion for robotics with hands-on learning and innovation. Below, I delve into the inspiration, process, challenges, and future plans for this project.

## Inspiration ✨

One of my most driving aspirations is to create a robot that can help people in their daily lives. This realization led me to discover my passion for robotics software development. Given the niche nature of this field and the lack of specific courses in school, I took it upon myself to explore the basics of robotics coding concepts over the past two months. This exploration began with working on PID algorithms, and it inspired me to create a ball balancing machine. The idea was to develop a project that could practically apply these concepts while also challenging my understanding and skills.

## The Process 🛠️

The project began with extensive research. I invested in stepper motors due to their high accuracy and quiet operation. The design needed a platform with at least two degrees of freedom (pitch and yaw) to angle the platform in any direction. Initially, I considered using a gimbal, but I encountered clearance issues with the motors. This led me to explore other manipulator designs, eventually settling on a 3-RPS parallel manipulator. A 3-RPS manipulator was ideal because it provided the necessary degrees of freedom, but I ultimately chose a varation of this manipulator (A 3-RRS manipulator platform) due to the lack of access to prismatic actuation.

Once the platform design was finalized, I began calculating the inverse kinematics necessary for coordinating platform tilt with stepper motor angles. This process involved advanced mathematical calculations, including solving complex equations related to the robot's geometry and movement. Each component—inverse kinematics values, resistive touchpad readings, motor control algorithms, and the PID control algorithm—was thoroughly tested. After a series of tests and refinements, I assembled and tuned the robot. The result is a finely tuned ball balancing machine, as demonstrated in the video below.

## Demo 🚀

[Watch the demo](https://)

## Built with 💻

Technologies used in the project:

- Arduino
- C++
- Stepper Motors

## Challenges I Ran Into 🚧

One of the biggest challenges was calculating the inverse kinematics. This complex calculation required a solid understanding of linear algebra and advanced mathematics. The process involved solving equations that define the relationship between the robot's physical structure and its movements. Specifically, it required deriving the mathematical model that translates the desired tilt of the platform into specific angles for the stepper motors. The complexity and precision required for these calculations pushed me to deepen my knowledge and skills in this area, making it one of the most challenging yet rewarding aspects of the project.

For those interested, a portion of the calculations performed for the inverse kinematics can be found [here](https://github.com/AnmeetS/Ball-Balancer/blob/main/Dynamic%20Plate%20Kinematics.pdf).

## What I Learned 📚

- Working with PID control algorithms
- Inverse kinematics calculations
- Stepper motor control and precision
- Integration of hardware and software components

## What's Next 🔮

- **Patterns:** Implementing code to allow the platform to trace patterns on the touchpad using the ball.
- **Interactivity:** Developing a frontend interface to let users specify platform actions.

## Stay Connected 📢

Thank you for visiting my repository! Your feedback and contributions are always welcome. Let's make something amazing together!

---

**Note:** This project is a continuous work in progress, and some features may be added soon. Stay tuned!
