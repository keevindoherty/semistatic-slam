/*
Potential change: For no persistence control, current map = mpst recently observed landmarks only? marginalize anything that hasn't been most recently observed

- Pose graph first factor = 0,0,0
- Relative odometry between consecutive factors (w/ angle as quaternion given in odometry file)
- no landmark information yet? how to incorporate that?
- All four functions (Generate Observations, LandmarkAssociation, UpdateLandmarkPredictions, UpdateGraphFactors, UpdatePersistence)

Generate Observations:
- Add each observation to landmarks vector
- Directly take data from 

*/

// void AddObservationToLandmarks (vector<Landmark*>& landmarks, vector<Observation>& observations, bool persistence){
//     //For each landmark observation from a point
//     //Determine "absolute" landmark location

//     //Add landmark to landmarks vector if it's not there before
//     if(!InLandmarks(pos, landmarks)){
//           LandmarkPersistence* l = new LandmarkPersistence(Symbol('l',landmarks.size()-1), pos, Book);
//           landmarks.push_back(l);
//     }

//     //FOR EACH OBSERVATION POINT:
//     //- generate new observation object
//     //- add to observations vector
//     //- return observation vector
// }

//LandmarkAssociation, no change

//UpdateLandmarkPredictions, no change

//UpdateGraphFactors, no change

//UpdatePersistence, 




