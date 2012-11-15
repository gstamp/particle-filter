(ns particlefilter.core
  (:use [particlefilter.draw]
        [quil core applet]))

;; General Particle Filter Algorithm
;;  Originally written by Martin J. Laubach in Python.
;;  Adapted to Clojure by Glen Stampoultzis

;; 1. Create (random) set of particles throughout the world
;; - These particles will have start with an initial weight of 1 (you might choose to start with a weight 1/samples instead)
;; 2. Robot makes an observation about some evidence.  
;; 3. We multiply the particle weights by how closely the particle matched the evidence.
;; 4. Next we normalize the weights.  ie, make them add up to 1.
;; 5. Resample the particles.  This step recreates all the particles based on the old ones.
;; - We take sample the old particles based on their weight.  Higher weighted particles are more likely to be selected.  We recreate new particles based on the selected particles resetting the weighting back to 1 (or whatever we chose as the starting weight)
;; 6. Add the motion of the robot + some noise to the resampled particles.

;; Notes: This program is impressively slow. Initially I thought this
;; was the drawing function but from my measurements it's the actual
;; calculation of the particles themselves. Unless you take special
;; care Clojure numerics can be extremely slow.
;;
;; It would be an interesting exercise to optimize this. The best way
;; to do this would probably be to convert everything over to
;; primatives and primative arrays and type hint appropriately.

;; (def maze-data [[2 0 1 0 0]
;;                 [0 0 0 0 1]
;;                 [1 1 1 0 0]
;;                 [1 0 0 0 0]
;;                 [0 0 2 0 1]])

;; (def maze-data
;;   [ [ 1 1 0 0 2 0 0 0 0 1 ]
;;     [ 1 2 0 0 1 1 0 0 0 0 ]
;;     [ 0 1 1 0 0 0 0 1 0 1 ]
;;     [ 0 0 0 0 1 0 0 1 1 2 ]
;;     [ 1 1 0 1 1 2 0 0 1 0 ]
;;     [ 1 1 1 0 1 1 1 0 2 0 ]
;;     [ 2 0 0 0 0 0 0 0 0 0 ]
;;     [ 1 2 0 1 1 1 1 0 0 0 ]
;;     [ 0 0 0 0 1 0 0 0 1 0 ]
;;     [ 0 0 1 0 0 2 1 1 1 0 ]])

(def maze-data
  [ [ 1 1 1 1 2 1 1 1 1 1 ]
    [ 1 2 0 0 1 1 0 0 0 2 ]
    [ 1 0 0 0 0 0 0 1 0 1 ]
    [ 1 0 0 0 1 0 0 0 0 1 ]
    [ 1 0 0 0 1 1 0 0 1 1 ]
    [ 1 0 1 2 1 0 0 0 2 1 ]
    [ 2 0 0 0 0 0 0 0 0 1 ]
    [ 1 0 0 0 0 0 1 0 0 1 ]
    [ 1 0 0 0 1 0 0 0 0 1 ]
    [ 1 0 1 1 1 2 1 0 1 1 ]])

(def particle-count 4000)

(def robot-has-compass false)

(defn rand-between [from to]
  (+ (rand (- to from))
     from))

(defn add-noise [level coords]
  (for [x coords] (+ x
                     (rand-between (- level) level))))

(defn add-little-noise [& coords]
  (add-noise 0.02 coords))

(defn add-some-noise [& coords]
  (add-noise 0.1 coords))

(def sigma2 (Math/pow 0.9 2))

(defn w-gauss [a b]
  (let [error (- a b)]
    (Math/pow (Math/E)
              (- (/ (Math/pow error 2)
                    (* 2 sigma2))))))

(defn distance [x1 y1 x2 y2]
  (Math/sqrt (+ (Math/pow (- x1 x2) 2)
                (Math/pow (- y1 y2) 2))))

(defn bisect-left [s x]
  "Similar to pythons bysect-left (I think) but slower"
  (count (take-while #(< % x) s)))

(defn compute-mean-point [particles]
  "Compute the mean for all particles that have a reasonable good weight.
   This is not part of the particle filter algorithm but rather an
   addition to show the 'best belief' for the current position"
  (let [m-count (reduce + (map :w particles))
        m-x     (reduce + (map *
                               (map :x particles)
                               (map :w particles)))
        m-y     (reduce + (map *
                               (map :y particles)
                               (map :w particles)))
        m-x     (/ m-x m-count)
        m-y     (/ m-y m-count)]
    (if (= m-count 0)
      [-1 -1 false]
      ;; Now compute how good that mean is -- check how many partciles
      ;; actually are in immediate vicinity
      (let [m-count (count (filter #(< (distance (:x %) (:y %) m-x m-y))
                                   particles))]
        [m-x m-y (> m-count (* particle-count 0.95))]))))

(defrecord WeightedDistribution [state distribution])

(defn create-weighted-distribution [state]
  (let [;;filtered-state (filter #(> (:w %) 0.0001) state)
        filtered-state state]
    (WeightedDistribution.
     filtered-state
     (reductions + (map :w filtered-state)))))

(defn pick [weighted-distribution]
  (try
    (let [rnd   (rand-between 0 1)
          index (bisect-left (:distribution weighted-distribution)
                             (rand-between 0 1))]
      (nth (:state weighted-distribution)
           index))
    (catch IndexOutOfBoundsException e
      (println "pick error")
      nil)))

(defrecord Particle [x y heading w noisy])

(defn create-particle
  ([x y] (create-particle x y nil 1 false))
  ([x y heading weight noisy]
     (if (not heading)
       (create-particle x y (rand 360) weight noisy)
       (if noisy
         (let [[x y heading] (add-some-noise x y heading)]
           (Particle. x y heading weight noisy))         
         (Particle. x y heading weight noisy)))))

(defn create-random [count maze]
  (take count (repeatedly #(apply create-particle (random-free-place maze)))))

(defn read-sensor [particle maze]
  "Find distance to nearest beacon"
  (distance-to-nearest-beacon maze (:x particle) (:y particle)))

(defn move-by [particle dx dy]  
  (assoc particle
    :x (+ dx (:x particle))
    :y (+ dy (:y particle))))

(defn advance-by [particle speed checker noisy]
  (let [[speed h] (if noisy
                    (add-little-noise speed (:heading particle))
                    [speed (:heading particle)])
        r         (Math/toRadians h)
        dx        (* (Math/sin r) speed)
        dy        (* (Math/cos r) speed)]

    (if (or (not checker) (checker particle dx dy))
      [true (move-by particle dx dy)]
      [false particle])))

(def robot-speed 0.2)
(defrecord Robot [x y heading w noisy step-count])

(defn choose-random-direction [robot]
  (assoc robot :heading (rand-between 0 360)))

(defn create-robot [maze]
  (let [[x y] (random-free-place maze)]
    (-> (Robot. x y 90 1 false 0)
        choose-random-direction)))

(defn read-robot-sensor [robot maze]
  "Poor robot, it's sensors are noisy and pretty strange,
   it only can measure the distance to the nearest beacon(!)
   and is not very accurate at that too!"
  (first (add-little-noise (read-sensor robot maze))))

(defn move [robot maze]
  "Move the robot.  Note that the movement is stochastic too"
  (let [[moved? new-robot] (advance-by robot
                                       robot-speed
                                       (fn [r dx dy] (is-free maze
                                                             (+ (:x robot) dx)
                                                             (+ (:y robot) dy)))
                                       true)]
    (if moved?
      new-robot
      (recur (assoc (choose-random-direction robot)
               :step-count (inc (:step-count robot)))
             maze))))

(defn shuffle-particles [particles robbie world]
  (let [nu             (reduce + (map :w particles))]
    (if (not= nu 0)
      (let [norm-particles (for [p particles] (assoc p :w (/ (:w p) nu)))
            dist           (create-weighted-distribution norm-particles  )]
        
        (for [_ particles]
          (let [p (pick dist)]
            (if p
              (create-particle (:x p) (:y p)
                               (if robot-has-compass
                                 (:heading robbie)
                                 (:heading p))
                               1
                               true)
              (first (create-random 1 world))))))
      particles)))

(def world (create-maze maze-data))
(def particles (atom (create-random particle-count world)))
(def robbie (atom (create-robot world)))

(defn reset []
  (reset! particles (create-random particle-count world))
  (reset! robbie (create-robot world)))

(defn draw-and-update []
  (try
    
    (draw-world world)

    (let [;; Read robbie's sensor
          robot-dist         (read-robot-sensor @robbie world)
          ;; Update particle weight accoring to how good every particle matches
          ;; robbie's sensor reading
          new-particles      (for [p @particles]
                               (if (is-free world (:x p) (:y p))
                                 (assoc p :w (w-gauss robot-dist
                                                      (read-sensor p world)))
                                 (assoc p :w 0)))
          
          ;; todo: best estimate for display
          ;;m_x, m_y, m_confident = compute_mean_point(particles)

          shuffled-particles (shuffle-particles new-particles @robbie world)

          old-heading        (:heading @robbie)
          new-robbie         (move @robbie world)
          d-h                (- (:heading new-robbie) old-heading)

          ;; Move particles according to my belief of movement (this may
          ;; be different than the real movement, but it's all I got)

          ;; broke
          final-particles    (for [p shuffled-particles]
                               (let [p          (assoc p :heading (+ (:heading p) d-h))
                                     [moved? p] (advance-by p robot-speed nil false)]
                                 p))]

      (show-particles world new-particles)
      ;; todo:
      ;; (show-mean m-x m-y m-confident)
      (show-robot @robbie world)

      (reset! robbie new-robbie)
      (reset! particles final-particles)

      )
    (catch Exception e
      (.printStackTrace e))))

;; Annoyingly defsketch always strts the sketch immediately so I've
;; wrapped it in a proc.
(defn start-sketch []
  (defsketch robbie-applet
    :title     "Poor robbie is lost"
    :renderer  :opengl
    :setup     #(setup world)
    :draw      draw-and-update
    :size      [600 600])  ; for some reason I can't reference
                           ; screen-size here.
  )

(defn -main []
  (start-sketch)
  )

(comment
  ;;(sketch)
  
  (run robbie-applet)
  (stop robbie-applet)
  (reset)
 )


