# A Microchip PIC-Compatible RISC CPU IP Core
## Introduction
The PIC10 IP Core is an instruction-compatible Verilog implementation of the Microchip PIC10F200-series microcontrollers. We chose to implement the PIC10F200-series CPU because of the small but effective RISC CPU architecture and the availability of free software development tools from Microchip. The PIC10F200-series of microcontrollers only has 33 instructions which makes the final hardware implementation compact. The CPU can address up to 512 instruction words which is small enough to fit on-chip but still large enough to be practically useful when programmed in assembly language. Figure 1 below shows the main modules of the PIC10 RISC CPU IP Core.
<img width="611" height="373" alt="image" src="https://github.com/user-attachments/assets/f87fe03c-d4b7-4898-936f-4b3b66495604" />
Figure 1. The main modules and interconnects in the PIC10 RISC CPU IP Core.

As figure 1 shows, the pic10 module contains the pic10_datapath and pic10_controller modules while the pic10_program_store is external to the CPU Core. This means that the Program Store can be located off-chip if needed. Locating the Program Store on-chip however enables the CPU Core to execute at the internal clock speed of the FPGA which results in very fast clock frequency compared to the original PIC10F200.
## Datapath Architecture
Figure 2 below shows a somewhat simplified datapath architecture where the eight Special Function Registers 0..7 are not shown. This datapath is sufficient to execute most instructions so it will be used to introduce the overall PIC10 CPU IP Core architecture.
<img width="616" height="335" alt="image" src="https://github.com/user-attachments/assets/e403c8dc-7614-4b5d-abf1-62b51a4fd59f" />
Figure 2. A simplified datapath that illustrates the basic datapath architecture.

Most instructions in the PIC10F200 architecture operate on either the 'W' or one of the 'f' registers. The 'W' register is input as the ALUs first operand while the currently addressed 'f' register is input as the second ALU operand. The ALU result is output on 'alu_bus' from where it is loaded into either the 'W' register or into the currently addressed 'f' register. The Controller either asserts the 'load_w_reg' or the 'load_ram_reg' depending on if the instruction word in the instruction Register directs the result to the 'W' or 'f' register respectively.

In order for the ALU to be able to update the STATUS register bits Z, C, and DC a separate 'status_bus' exists together with three control signals named 'load_z', 'load_d' and 'load_dc'. These signals are displayed below in figure 3. Also note that figure 3 contains the 'ALU Mux' on operand input two which allows the Controller to select the second ALU operand to come from either the RAM Registers (shown above in figure 2) or from one of the eight SFR Registers 0..7.
<img width="613" height="340" alt="image" src="https://github.com/user-attachments/assets/c0fef944-b841-498d-b3d1-5e9f054a30d3" />
Figure 3. The actual datapath surrounding the ALU.

The ALU knows from the 'ir_reg_bus' which instruction word is currently being executed. The 'carry_bit' input is connected to the C bit in the STATUS register. This is used during additions and subtractions. The 'ram_data_bus' input to the ALU Mux comes from the RAM registers as shown above in figure 1. The 'sfr_data_bus' comes from the currently addressed SFR register.

The PIC10 architecture allows both direct and indirect addresses. A direct address is encoded directly in the 12-bit instruction word while an indirect address is taken from the FSR Special Function Register. The address generation logic is shown below in figure 4.
<img width="610" height="347" alt="image" src="https://github.com/user-attachments/assets/0dbd58bd-f90b-4f45-b45f-90be7c405474" />









