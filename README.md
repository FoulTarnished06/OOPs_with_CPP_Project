# OOPs_with_CPP_Project
This is the project for my oops with cpp class.
# 2D Rigid Body Physics Engine 🪐

A real-time, object-oriented 2D physics engine built from scratch using C++ and the SFML graphics library. This project simulates Newtonian mechanics, particle kinematics, and elastic collisions for rigid bodies. 

This project was developed to demonstrate core Object-Oriented Programming (OOP) principles alongside applied numerical methods.

## Features
* **Real-Time Dynamics:** Simulates gravity, velocity, and acceleration using Newton's Second Law of Motion.
* **Numerical Integration:** Implements Semi-Implicit Euler integration for highly stable, frame-rate independent physics calculations.
* **Collision Detection:** Accurate circle-to-circle and boundary intersection checking.
* **Collision Resolution:** Mathematically resolves overlaps (penetration depth correction) and calculates dynamic impulses based on the Law of Conservation of Momentum and Restitution (bounciness).
* **Interactive Sandbox:** Users can spawn new physical bodies dynamically in real-time using mouse inputs.

## Object-Oriented Architecture
The engine is strictly designed using C++ OOP paradigms:
* `Vector2D`: A custom mathematical struct encapsulating 2D vector arithmetic (dot products, magnitude, normalization).
* `RigidBody`: Encapsulates the physical state of an entity (mass, radius, velocity, position) and handles its discrete time-step updates.
* `PhysicsWorld`: The central mediator class that manages the collection of bodies, applies global forces (gravity), and executes the $O(n^2)$ collision detection loop.

## Technologies Used
* **Language:** C++14 / C++17
* **Graphics API:** SFML (Simple and Fast Multimedia Library) 2.5.1
* **Compiler:** MinGW GCC 7.3.0 (64-bit)
* **Environment:** Visual Studio Code

## How to Run

### Prerequisites
You must have **SFML 2.5.1** and a compatible **g++ compiler (e.g., 7.3.0)** installed on your system.

### Compilation
If you are compiling via the terminal, use the following command (ensure your `-I` and `-L` paths match your SFML installation directory):

g++ main.cpp -I"C:\SFML-2.5.1\include" -L"C:\SFML-2.5.1\lib" -lsfml-graphics -lsfml-window -lsfml-system -o physics_engine.exe

### Execution :- 

Ensure all SFML .dll files (from the SFML/bin folder) are located in the same directory as your compiled .exe.

Run the executable:

* **./physics_engine.exe**
