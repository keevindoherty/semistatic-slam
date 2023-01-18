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

/*
Meeting 1/18 notes:
- Developed control
    - Refactored into abstract classes and used that to write
    - See control potential change
- More work w data association in persistence model, realized one major flaw that was fixed to make model more robust
- Questions on data?? Show odometry vs poses quaternion graphs and ask
- Have plan for data persistence functions and data non persistence functions, and hopefully some work on both of these by tmrw
*/

void AddObservationToLandmarks (vector<Landmark*>& landmarks, vector<Observation>& observations, bool persistence){
    //For each landmark observation from a point
    //Determine "absolute" landmark location

    //Add landmark to landmarks vector if it's not there before
    if(!InLandmarks(pos, landmarks)){
          LandmarkPersistence* l = new LandmarkPersistence(Symbol('l',landmarks.size()-1), pos, Book);
          landmarks.push_back(l);
    }

    //FOR EACH OBSERVATION POINT:
    //- generate new observation object
    //- add to observations vector
    //- return observation vector
}

//LandmarkAssociation, no change

//UpdateLandmarkPredictions, no change

//UpdateGraphFactors, no change

//UpdatePersistence, 

/*
- Object extraction
- 
*/



