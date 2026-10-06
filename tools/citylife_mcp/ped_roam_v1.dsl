; BP_CityPed graphs as found in CityLife_Day on 2026-09-29 (read_graph_dsl).
; The script that wrote them was never committed; this is the record, and the
; rollback for tools/citylife_mcp/ped_walk.py.
; variables: HomeLocation, RoamRadius, bUseNavMesh, Route, Idx, bWaiting, NextMoveTime, Goal

; ---- Roam (function; a 1 s looping timer from BeginPlay when bUseNavMesh)
(fn Roam ()
  (bind _returnvalue (Utilities|Time|GetGameTimeInSeconds))
  (if (< (Math|Vector|VectorLength (Transformation|GetVelocity)) 10.0)
    (if (>= _returnvalue (Variables|Default|GetNextMoveTime))
      (if (not (|GetbWaiting))
        (|SetbWaiting true)
        (Variables|Default|SetNextMoveTime (+ _returnvalue (select (Math|Random|RandomBoolWithWeight 0.7) (Math|Random|RandomFloatInRange 0.2 1.0) (Math|Random|RandomFloatInRange 2.0 6.0))))
        (else
          (Variables|Default|SetGoal (select (AI|Navigation|ProjectPointtoNavigation (select (> (Math|Vector|Distance(Vector) (Transformation|GetActorLocation) (Variables|Default|GetHomeLocation)) (Variables|Default|GetRoamRadius)) (Variables|Default|GetHomeLocation) (Math|Vector|vector+vector (Transformation|GetActorLocation) (Math|Vector|MakeVector (* (.x (Math|Random|RandomUnitVectorInConeInDegrees (Transformation|GetActorForwardVector) (select (Math|Random|RandomBoolWithWeight 0.8) 30.0 180.0))) 1400.0) (* (.y (Math|Random|RandomUnitVectorInConeInDegrees (Transformation|GetActorForwardVector) (select (Math|Random|RandomBoolWithWeight 0.8) 30.0 180.0))) 1400.0)))) 0 0 (Math|Vector|MakeVector 500.0 500.0 250.0)) (AI|Navigation|ProjectPointtoNavigation (select (> (Math|Vector|Distance(Vector) (Transformation|GetActorLocation) (Variables|Default|GetHomeLocation)) (Variables|Default|GetRoamRadius)) (Variables|Default|GetHomeLocation) (Math|Vector|vector+vector (Transformation|GetActorLocation) (Math|Vector|MakeVector (* (.x (Math|Random|RandomUnitVectorInConeInDegrees (Transformation|GetActorForwardVector) (select (Math|Random|RandomBoolWithWeight 0.8) 30.0 180.0))) 1400.0) (* (.y (Math|Random|RandomUnitVectorInConeInDegrees (Transformation|GetActorForwardVector) (select (Math|Random|RandomBoolWithWeight 0.8) 30.0 180.0))) 1400.0)))) 0 0 (Math|Vector|MakeVector 500.0 500.0 250.0)) (AI|Navigation|GetRandomReachablePointinRadius (Transformation|GetActorLocation) 1200.0)))
          (|SetbWaiting false)
          (Variables|Default|SetNextMoveTime (+ _returnvalue 1.5))
          (AI|Navigation|SimpleMoveToLocation (Pawn|GetController) (Variables|Default|GetGoal)))))))

; ---- EventGraph
(event EventBeginPlay
  (Variables|Default|SetHomeLocation (Transformation|GetActorLocation))
  (if (|GetbUseNavMesh)
    (Utilities|Time|SetTimerbyFunctionName self "Roam" 1.0 true)))

(event Collision|EventActorBeginOverlap (OtherActor))

(event EventTick (DeltaSeconds)
  (bind _route (Variables|Default|GetRoute))
  (bind _returnvalue (Utilities|Array|Length _route))
  (bind _returnvalue_1 (Transformation|GetActorLocation))
  (bind _idx (Variables|Default|GetIdx))
  (bind _output (Utilities|Array|Get(acopy) _route _idx))
  (bind _returnvalue_2 (Math|Vector|vector-vector _output _returnvalue_1))
  (bind _x (.x _returnvalue_2))
  (bind _y (.y _returnvalue_2))
  (bind _returnvalue_3 (Math|Vector|MakeVector _x _y))
  (if (not (|GetbUseNavMesh))
    (if (> _returnvalue 1)
      (if (< (Math|Vector|VectorLength _returnvalue_3) 80.0)
        (Variables|Default|SetIdx (Math|Integer|%(Integer) (+ _idx 1) _returnvalue))
        (else
          (Pawn|Input|AddMovementInput (Math|Vector|Normalize _returnvalue_3)))))))
