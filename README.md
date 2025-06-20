# RISC-Processor


# Design & Implementation of RISC Processor

The aim of this project is to design a 32 bit single cycle RISC processor. The processor will be able to execute small programs written in assembly language. We will be using bottom up design methodology for this project. A processor is made up of several components like ALU, control unit, program counter, register bank etc. We will design these individual modules using verilog programming language. Finally we will instantiate them under top level module to complete the processor design. With this we complete front end part of VLSI design. Now we will simply import verilog code inside cadence tool. We will perform IO planning, floor planning, power planning, placement & routing. With this we complete back end part of VLSI design.

## Software Tools
| Tool        | Description |
| ----------- | ----------- |
| ![note-2.jpg](https://i.postimg.cc/sgZFkb9V/note-2.jpg) | Vivado is used for programming FPGA and SoC devices. It offers tools for synthesis, implementation, simulation, and verification of digital designs. It supports various languages like Verilog, VHDL, and SystemVerilog.  |
| ![Cadence-Emblem.png](https://i.postimg.cc/dty4wFXS/Cadence-Emblem.png)| Cadence Design Systems provides a wide range of products and services including digital integrated circuit design, functional verification, custom IC design, PCB layout and design, and system design enablement. |

## specifications of our processor

| Sr. No.        | Specification |
| ----------- | ----------- |
| 1  | 32 bit processor |
| 2  | 5 stages fetch, decode, execute, memory, write back|
| 3  | 32 bit long instruction |
| 4  | 8 GPR inside register bank |
| 5  | 8 bit flag register |
| 6  | 16 bit synchronous Program counter |
| 7  | 64 Kb of program memory (ROM) |
| 8  | 64 Kb of data memory (RAM) |
| 9  |opcode is 7 bit, max 2^7 = 128 types of instruction |
| 10 | currently 25 instruction  |
| 11 | 1:5 BUS decoder |
| 12 | 2:1 BUS multiplexer |

## circuit diagram 
![Circuit-Diagram.jpg](https://i.postimg.cc/nrW6HX06/Circuit-Diagram.jpg)

## ISA (instruction set architechture)
![ISA-image.png](https://i.postimg.cc/6pjpdVGv/ISA-image.png)

## xilinx vivado output - 
### 1st is schematic
### 2ns is waveform window
### 3rd is TCL window
![xilinx vivado output](https://postimage.me/images/2025/06/13/Screenshot-3868.png)

## cadence output
### 1st is schematic in vivado
### 2nd is netlist on genus
### 3rd is layout on innovus
![cadence output](https://postimage.me/images/2025/06/13/Screenshot-3871.png)

## To create IO pad ring we exported layout from innovus to virtuoso
![IO pad ring](https://postimage.me/images/2025/06/13/Screenshot-3872.png)

## Team Members
- [@vansh](https://github.com/VanshD40)
- [@ansih](https://github.com/KNIGHTAPG)
- [@sukanya](https://github.com/sukanyap1)
- [@godkar](https://github.com/Atharva-Godkar)