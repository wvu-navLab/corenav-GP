<!-- <p align="center">
<img alt="Architecture" src="doc/corenavICE.gif" width="300">
</p> -->

A comparison of robust kalman filters for improving wheel-inertialodometry in planetary rovers

Methods Implemented :

huberUpdate()  // C.  D.  Karlgaard,  “Nonlinear  regression  huber–kalman  filtering  andfixed-interval smoothing,” 2015. Link: https://arc.aiaa.org/doi/full/10.2514/1.G000799?casa_token=vJibT0LXw48AAAAA:xFXdIWjlgYGfgTpuOERFESNJXJdbVxEW1Xtz9FCeIJPFbH2SG4mTcKfAO3tJeerMWdnjZR_sIUNO

normalUpdate()   // C.  Kilic,  J.  N.  Gross,  N.  Ohi,  R.  Watson,  J.  Strader,  T.  Swiger,S.  Harper,  and  Y.  Gu,  “Improved  planetary  rover  inertial  navigationand  wheel  odometry  performance  through  periodic  use  of  zero-typeconstraints, ” 2019 Link: https://arxiv.org/pdf/1906.08849

orkf1Update()   // G.  Agamennoni,  J.  I.  Nieto,  and  E.  M.  Nebot, "Approximate inference in state-space models with heavy-tailed noise," Link : https://ieeexplore.ieee.org/iel5/78/4359509/06266757.pdf?casa_token=wgvggTT8YwsAAAAA:LOLzYFTTlcgaUUktIRNWbkGIzKDU6Fu-3RQRI4J_hrsTrJHQDerhpp7uCZuhzbrqBOCvESg6NRs

orkf1DriftUpdate()  // Drifting noise version of the above filter

orkf2Update()  // R.  Pich ́e,  S.  S ̈arkk ̈a,  and  J.  Hartikainen,  “Recursive  outlier-robustfiltering  and  smoothing  for  nonlinear  systems  using  the  multivariatestudent-t  distribution,” 2012 Link : https://ieeexplore.ieee.org/iel5/6335571/6349703/06349794.pdf?casa_token=h4pznoRW7n0AAAAA:TRyNS1-_xEdr3jBUrzdrrdMAMOm7SeitRG7MajSFiEf2wmvjZnyhMt7Cm2ACugP8dIttzjGpmMY

orkf3Update()  // Y. Huang, Y. Zhang, Z. Wu, N. Li, and J. Chambers, “A novel adaptivekalman  filter  with  inaccurate  process  and  measurement  noise  covari-ance  matrices,” 2017 Link : https://ieeexplore.ieee.org/iel7/9/4601496/08025799.pdf

orkf4Update() (CSKF)  // G.  Chang,  “Robust  kalman  filtering  based  on  mahalanobis  distanceas  outlier  judging  criterion,” 2014 Link: https://idp.springer.com/authorize/casa?redirect_uri=https://link.springer.com/content/pdf/10.1007/s00190-013-0690-8.pdf&casa_token=cYCuvYdFiUEAAAAA:oSSjiLZa-u05h2yx0XrxWpkmPid-Z2Jqan4qQsUW27SrmjqvifC3jMTd0PAdQOnXGifMTO0JyPmayvPONA

orkf5Update() (AEKF)  // S.  Akhlaghi,  N.  Zhou,  and  Z.  Huang,  “Adaptive  adjustment  of  noisecovariance in kalman filter for dynamic state estimation,” 2017   https://arxiv.org/pdf/1702.00884.pdf

orkf6Update()  // Y. Yang, H. He, and G.-c. Xu, “Adaptively robust filtering for kinematicgeodetic  positioning,” 2001 Link : https://idp.springer.com/authorize/casa?redirect_uri=https://link.springer.com/content/pdf/10.1007/s001900000157.pdf&casa_token=teeL1FckHyIAAAAA:GLCpJEv9jGpEedViMSSYuTcHcGjNq6_zK45N-pYlDodp-g07bRQQE2HUf_nTMEYasLCwnOf6wNKWg0oyZg
