(define (domain bioloid)

(:types encoder)

(:constants E0 E1 E2 E3 E4 E5 E6 E7 E8 E9 E10 E11 E12 E13 E14 E15 E16 E17 E18 - encoder)

(:functions (value ?e - encoder) (steps) (last))
  
(:action action0
  :precondition (and (>  (+ (value E17) -20) -1) (<  (+ (value E17) -20) 301)
		      (>  (+ (value E7) -40) -1) (<  (+ (value E7) -40) 301)
		)
  :effect
  (and (increase (value E17)  -20) (increase (value E7) -40) (increase (steps) 1) (assign (last) 0))
)


(:action action1
  :precondition (and (>  (+ (value E0) 20) -1) (<  (+ (value E0) 20) 301)
		      (>  (+ (value E8) 40) -1) (<  (+ (value E8) 40) 301)
		      (>  (+ (value E10) 20) -1) (<  (+ (value E10) 20) 301)
		)
  :effect
  (and (increase (value E0) 20) (increase (value E8) 40) (increase (value E10) 20) (increase (steps) 1) (assign (last) 1))
)

(:action action2
  :precondition (and (>  (+ (value E6) 20) -1) (<  (+ (value E6) 20) 301)
		      (>  (+ (value E7) 20) -1) (<  (+ (value E7) 20) 301)
		      (>  (+ (value E8) -60) -1) (<  (+ (value E8) -60) 301)
		)
  :effect
  (and (increase (value E6) 20) (increase (value E7) 20) (increase (value E8) -60) (increase (steps) 1) (assign (last) 2))
)

  (:action action3
  :precondition (and (>  (+ (value E12) 30) -1) (<  (+ (value E12) 30) 301)
		      (>  (+ (value E13) 50) -1) (<  (+ (value E13) 50) 301)
		)
  :effect
  (and (increase (value E12) 30) (increase (value E13) 50) (increase (steps) 1) (assign (last) 3))
)
  
  (:action reverse
  :effect
  (and (when (and (= (last) 0)) (and (increase (value E17)  20) (increase (value E7) 40) (increase (steps) 1) (assign (last) 0)))
  (when (and (= (last) 1)) (and (increase (value E0) 20) (increase (value E8) 40) (increase (value E10) 20) (increase (steps) 1) (assign (last) 1)))
  (when (and (= (last) 2)) (and (increase (value E6) 20) (increase (value E7) 20) (increase (value E8) -60) (increase (steps) 1) (assign (last) 2)))
  (when (and (= (last) 3)) (and (increase (value E12) 30) (increase (value E13) 50) (increase (steps) 1) (assign (last) 3))))
)

)