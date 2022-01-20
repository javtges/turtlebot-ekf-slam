# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.

   - Which of the methods would you implement and why?


   Three different ways to implement the ~normalize~ funcionality:

   1. Within a class or struct as a member: This requires a class for the method to be inside (ex. a Vector class), which could be useful if normalizing is a related function to other members of the class.
   2. In its own standalone function: This requires the least supporting code, no other classes to exist, however it cannot utilize the functionality of any other supporting functions.
   3. Put it in its own class: This would involve creating a normalize class, which could be useful if there are other related functions to put in the class (ex. different methods for measuring vector norms). Being in a class, this also protects data from unintentional access as classes by default set data to private.

        For this turtlelib package, option 2 of implementing normalize as a standalone function makes the most sense. All that is required is one argument of Vector2D, where no data is required to be private and there is no Vector2D class to put the function into. Therefore a standalone function is the preferred option.


2. What is the difference between a class and a struct in C++?

    * The biggest difference between a class and a struct in C++ is that the members of a class are by default private, whereas in a struct they are public. This is shown in the rigid2d files, where specific functions were made to return components of a Transform2D class object to get around this difference, where members of Twist2D and Vector2D were able to be accessed at any time.


3. Why is Vector2D a struct and Transform2DClass (refer to at least 2 specific C++ core guidelines in your answer)?

    * One reason that Vector2D is a struct rather than a class is because all the vector components can vary independently - whereas in a transformation matrix changing any of the values changes the overall matrix. This refers to C++ core guideline C.2.

        A second reason why Vector2D is a struct is because we may need to extract the components from the data type by themselves (referring to C.1). However, Transform2D is a class because we have little intention to reference the individual matrix entries on their own, and it could be dangerous if individual entries could be edited. This is referencing guideline C.9, stating that in a class, exposure to members should be minimized.


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

    * As per C++ guideline C.46, single-argument constructors should be labeled as explicit, to avoid unintended conversions. This may be useful in our turtlelib case as a Transform2D can be initialized with multiple single datatypes, or a combination of those datatypes.


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

    * Per C++ guideline Con.1, all objects should be immutable (const) unless they have explicit reason not to be. So, as inv() returns a new Transform2D object, it should be and is declared const. However, Transform2D::operator*=() modifies the first argument of the matrix multiplication, and returns the original modified first argument of the operator, so there is a reason not to follow Con.1 and declare as const.
    

# Sample Run of frame_main
```
Enter transform T_{a,b}:
deg: 90 x: 0 y: 1
Enter transform T_{b,c}:
deg: 90 x: 1 y: 0
T_{a_b}: deg: 90 x: 0 y: 1
T_{b_a}: deg: -90 x: -1 y: -6.12323e-17
T_{b_c}: deg: 90 x: 1 y: 0
T_{c_b}: deg: -90 x: -6.12323e-17 y: 1
T_{a_c}: deg: 180 x: 6.12323e-17 y: 2
T_{c_a}: deg: -180 x: -1.83697e-16 y: 2
Enter vector v_b:
1 1 
v_bhat: [0.707107 0.707107]
v_a: [-1 2]
v_b: [1 1]
v_c: [1 1.11022e-16]
Enter twist V_b:
1 1 1
V_a [1 0 1]
V_b [1 1 1]
V_c [1 2 -1]
```