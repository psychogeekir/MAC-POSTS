## MAC-POSTS

[![DOI](https://zenodo.org/badge/52655219.svg)](https://zenodo.org/badge/latestdoi/52655219)

### Mobility Data Analytics Center - Prediction, Optimization, and Simulation toolkit for Transportation Systems

In the codes, we also note it as MINAMI (MNM), which represents Multi-functIonal dyNAmic network Modeling tookIt

#### Introduction

MAC-POSTS is the abbreviation of Mobility Data Analytics Center - Prediction, Optimization, and Simulation toolkit for Transportation Systems. It is an open-source mesoscopic dynamic traffic simulation toolkit for large-scale transportation networks. Its core traffic simulation module is written in C++ for the performance purpose while it also provides convenient Python-APIs for input and output data processing.

#### What MAC-POSTS can do?

MAC-POSTS can be used in many dynamic transportation network modeling tasks, such as dynamic network loading and dynamic traffic assignment. Particularly, it has the following highlighted features:

- High spatial and temporal granularity
    - MAC-POSTS can produce second-by-second vehicle traces through the entire trip.
- Large-scale simulation
    - Compared to existing off-the-shelf dynamic models, MAC-POSTS can handle much larger networks (e.g., regional networks with tens of thousands of links and origin-destination (O-D) pairs and millions of vehicles)
- High fidelity
    - The dynamic traffic O-D demands for the simulation can be calibrated efficiently using our machine-learning-based dynamic O-D demand estimation algorithm, which minimizes discrepancies between simulated data and observed data in the real world.
- Multi-source data
    - MAC-POSTS can utilize traffic data from multiple sources, such as traffic counts, travel speeds, bus transit, and parking.  
- Multi-class vehicles
    - MAC-POSTS can handle multi-class vehicles (e.g., cars and trucks) while the existing dynamic models often assume only single class vehicles exist.
- Rich travel modes and behavior
    - MAC-POSTS provides more travel modes and behavior models for simulation, e.g., driving, bus transit, mobility services, and parking and ride.
- Traffic and emission metrics
    - Based on the high-granularity simulation, MAC-POSTS can produce many useful metrics related to traffic, fuel consumptions, and emissions.


#### Contributors

- Wei Ma
- Xidong Pi
- Qiling Zou
- Sean Qian (Advisor)

#### Contact Info

For any questions, please contact zql0419@gmail.com

#### Copyright

Copyright 2014-2022 Carnegie Mellon University

#### Acknowledgements
This project is funded in part by Traffic 21 Institute, Carnegie Mellon University's Mobility21, Technologies for Safe and Efficient Transportation (T-SET) and US Department of Transportation. The contents of this project reflect the views of the authors, who are responsible for the facts and the accuracy of the information presented herein. The U.S. Government assumes no liability for the contents or use thereof.
