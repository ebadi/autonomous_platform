## Shared libraries

This folder is meant to collect libraries shared by multiple hw nodes.

These can then be used by multiple platformIO projects by including the path in the platform.ini file for the active project.

### Structure of shared library

The structure of a shared library shall be split into two parts, the code itself located in the **src** directory and a documentation folder **documentation_and_research**:

```bash
  shared_library_example
 ┣  src : Contains the code that is imported into projects.
 ┃ ┣    some_code.cpp
 ┃ ┣    some_code.h
 ┃ ┗    ....
 ┃
 ┗  documentation_and_research : Contains relevant documentation for the code presented in a .md README file
   ┣   Images (Optional)
   ┣   Manuals (Optional)
   ┣   Test_rig_code (Optional) : Contains any related code that
   ┗    README.md
   ┃  
   ┣   SPCU
   ┗   README.md
```
