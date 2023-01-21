# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality

      Design 1)

      Vector2D normalize(const Vector2D* vec)
      {
         double mag{};
         Vector2D n_vec;
         mag = sqrt(vec.x*vec.x + vec.y*vec.y);
         n_vec.x = vec.x/mag;   
         n_vec.y = vec.y/mag;
         return n_vec;
      }


      Design 2)

      Vector2D normalize(const Vector2D vec)
      {
         double mag{};
         Vector2D n_vec;
         mag = pow(pow(tw.x, 2) + pow(tw.y, 2), 0.5);
         n_vec.x = vec.x/mag;
         n_vec.y = vec.y/mag;
         return n_vec;
      }


      Design 3)

      Vector2D normalize(Vector2D vec)
      {
         double mag{};
         mag = pow((pow(vec.x,2) + pow(vec.y,2)), 0.5);
         vec.x /= mag;
         vec.y /= mag;
         return vec;
      }


   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.

      Design 1)
      Pros:
         -Uses pointer so doesn't need to make a copy of the value to run the function.
         -Uses const keyword with pointer to guarantee no change of the value in memory.

      Cons: 
         Have to create new object, increasing overhead and memory usage.
         Have to create pointer variable, increasing chance of value change.
         

      Design 2)
      Pros:
         -Uses the input parameter uses const keyword to indicate that it will not be changed (immutable).

      Cons:
         -Can't do anything to vec so you have to create and initialize a new struct, increasing computation time and memory usage.


      Design 3)
      Pros:
         -Don't have to create and initialize a new Vector2D struct, can just return the parameter after it's been acted on.
        
      Cons:
         -This may cause confusion since the function itself looks like it's changing the object's vec data, however it's just using the local variable.


   - Which of the methods would you implement and why?

      I would implement Design 3 - don't have to worry about creating pointers or new objects


2. What is the difference between a class and a struct in C++?

   The difference is the default accessibility of its members methods. They are public in a struct while private in a class. Other than that there are no differences.


3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?

   Vector2D's variables are public and therefore can be a struct by default, while Transform2D has private variables.


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

   They are single argument constructors, which should be made explicit according to C++ core guidelines to avoid unintended conversions.


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

   inv() is declared constant to guarantee that we do not change the object's member variables within the function. operator*= is not because we are directly altering the internals of the left hand side of the operation, using const would prevent that and throw an error.