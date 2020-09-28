# Theme 3 Sprint 2 Robot Documentation

The purpose of this document is to record key information about the design of the robot, any lessons from the sprint and to help other themes use the platform for further testing once the sprint is complete. 

- Version 0.0.1
	- My [NF] first attempt at documenting the mechanical, electrical and software aspects of Sprintbot, from a Theme 3 perspective. 

## Sprint 2
Sprint 2 was a 4 week long (26th Feb - 24th March 2020) hardware focused sprint, starting with a 2 day meeting and ending with a single day integration push, both at ICAIR. 
Due to the Covid-19 pandemic the end of the sprint was postphoned until access to campuses was partically restored (October 2020).
The briefing document is [here](https://drive.google.com/drive/folders/1twYxoXcRTZItLzTB_jrvbxy3_hq5Eg7F)

## Nomenclature
The robot developed in Sprint 2 of the Pipebots project may be refered to as _SprintBot_ or colloquially _BigBallBot_ 
>Note: Some members call it _Sprintbot-2_ which maybe confusing as this was the first robot developed in a sprint, but was undertaken in the second sprint event of the project. 

# Software

## Robot Firmware
## Android App
# Electronics
## T3 Essentials
## Other Theme Payloads

# Hardware
>This part is not currently intended to be a complete reference but maybe added to in the future


### To Do
- Add Solidworks file
- Add Folder of final STLs
- Add images of first version 
- Add technical details such as size
- Explain drive system inc key bits to tight

## Development
During the initial meeting a conceptual design was agreed upon. The robot was to be ball shaped to allow it to fit into pipes and go around corners while maximising the internal space avaliable for payloads. 

![Initial Concept](https://lh3.googleusercontent.com/E7FioxXoL-sD-zDWrzA2RGpfFI7eF6nUFwQqx7UFxy_vZ_R5mDplhw4VAzh1OrHBHdupwxcmBbxF3go631b102Iw8U5Eb_4meXYR49-zBM7Rpwbfli8-9HHBrAoKDmLpVV-MdVv-RIjK5_oDu73OYlal7EFl6ujYleHDW7PzoO9_DiL66SersYBGapeCW-zjARIoLfbAff85Z5uSwGCnJ6UfuGmzIgYy5cr2w-85sbtnEkRcoOqBmJ2PNo91SmD7UVmtRZCSa1e1rhqks-aH1PXL2_OYJlbifZepETJRkPieoGwq9qrOeQK899fbBwwYsVApV-OtX-GWENCOV0HSty17_B4rmZEjMODti0sJXOuepSgcVokiA7k3IAFGY6MdSWqVNeOgyv5riF4kSS4XssJ0UhxEyQATmiHHvBzcDITr3sixGHQLD-9bXG4Do2lByJ8U9-gooKx292SVkNmjKZZQAER7MCIWl7CbByryLzN-XIRIkPgXkFhT7RIsjQAf4996Ha25CRKdyi0yA6r8Edinmpq1aeVNKFthkhn5x-OHUbaQOrzhVEQjoyJHERmY5JnDAb-ciIH6_cDwhPEhnbCqO4OJKHAlBHevrB4U9uHOTMYQMACrEDAvn2SGZlCcncxcuuIp543qxzjRJNtHTx3yFrz1QTdWdGeDkoc1Hi-_-tomOfG0MT0N5orx=w712-h607-no?authuser=0 =400x350)
*The initial Concept*

![Final Render](https://leeds365.sharepoint.com/sites/TEAM-EPSRCPipebots/Shared%20Documents/Sprint%202/Photos%20and%20Videos/FancyRender.jpg =400x400)
*Final Render*

![enter image description here](https://leeds365.sharepoint.com/sites/TEAM-EPSRCPipebots/Shared%20Documents/Sprint%202/Photos%20and%20Videos/IMG_20200826_094830%20b.jpg =400x400)
*Final Robot*

Designing, manufacturing, procuring and assembling the components required for a mobile robot designed to go into a real outdoor environment is a very difficult task in the time constrains of a sprint. 
To achieve it we first decided upon the components that must be bought externally (motors, motor drivers gears etc.) before finialising our mechanical design while these were on order. Procuring specialist components in a short timeframe is a challenge that shoud not be overlooked in future sprints and it must be accepted that the costs will rise due delivery costs and the potential for design changes to make parts obsolete. 
For the most part this was avoided, but it was decided to 3D print the gears rather than use the machined ones that were ordered as this gave a much greater flexibility in design and considerablly reduced the weight. 
This allowed the large ring gears to have a larger hole in the centre, giving more space for other components and the outersurface was keyed to allow the tyre to grip when overmoulded. The bevel drive gears now have much larger teeth giving greater robustness and shaft hole is easily varied and D shaped as required. 

3D printing was used to manufacture the whole mechanical structure of the robot as it has the shortest leadtime of the methods avaliable to us, and allows great geometric freedom. Fused Filament Fabrication was used with an enhanced PLA material (Polymaker Polymax) as this is capable of producing parts that are functional and tough enough for testing use when producing parts of this size. 

The robot was designed such that parts are side-invariant, i.e. they can be used on either side of the robot. This was decided to reduce complexity and repetitive design and means that a smaller number of spare parts need to be on hand. The robot only uses 4 major parts per side, allowing simple assembly. 

<!--stackedit_data:
eyJoaXN0b3J5IjpbLTgyMjcyODI0MCwtMjEyOTEyNTcwNCwxMT
k5NjMwNTM1LDEzNjA2MDY0XX0=
-->