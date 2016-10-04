# bOb Prototype Git Repo

## Introduction

This git repo contains the prototype development code for the new bObsweep system. The code implements many different algorithms, including:

* SLAM algorithms
* Control systems
* Navigation algorithms
* Map Storage Structure
* Frontier detection and exploration
* Coverage Algorithms

## Generating documentation

To generate documentation, make sure you have Doxygen installed and then cd into the doxygen folder. Run the command 'sh generate_documentation.sh'. This command will generate the documentation at 'doxygen/generated_documentation'. You can then go into the generated folder and open 'doxygen/generated_documentation/html/index.html' in the browser of your choice. It's also recommended that you create a bookmark to the documentation in your browser so that you can easily navigate to it again. This folder is ignored in .gitignore so it won't be added to git despite the fact that it is generated inside the git directory.
