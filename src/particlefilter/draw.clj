(ns particlefilter.draw
  (:use [quil.core]))

(def screen-size [600 600])

(defrecord Maze [maze width height blocks update-cnt one-px beacons])

(defn create-maze [maze]
  (let [width      (count (first maze))
        height     (count maze)
        update-cnt 0
        one-px     (/ (first screen-size) width)
        ;; eww - world structure needs some abstraction
        blocks     (filter identity
                           (map-indexed
                            (fn [i block]
                              (let [x (mod i width)
                                    y (int (/ i width))
                                    nb-y (- height y 1)]
                                (if (not= block 0)
                                  [x nb-y])))
                            (flatten maze)))
        beacons    (apply concat
                          (filter identity
                                  (map-indexed
                                   (fn [i block]
                                     (let [x (mod i width)
                                           y (int (/ i width))
                                           nb-y (- height y 1)]
                                       (if (= block 2)
                                         [[x nb-y] [(inc x) nb-y]
                                          [x (inc nb-y)] [(inc x) (inc nb-y)]])))
                                   (flatten maze))))]
    (Maze. maze width height blocks update-cnt one-px beacons)))

(defn setup [maze]
  nil)

(defn draw-world [maze]
  ;;(scale (:one-px maze))
  (background-int (* 255 155 200))
  (fill-int (color 220 102 0))
  (rect-mode :corners)
  (color 255 0 0)
  (let [s (:one-px maze)]
    (doseq [[x y] (:blocks maze)]
      (rect (* x s) (* y s)
            (* (inc x) s) (* (inc y) s)))
    (fill-int (color 200 200 0))
    (doseq [[x y] (:beacons maze)]
      (ellipse (* s x) (* s y) 10 10)
      )
    ))

(defn show-particles [maze particles]

  (doseq [{x :x y :y h :heading} particles]
    (fill-int (color 90 0 180))
    (let [s (:one-px maze)
          xx (* x s)
          yy (* y s)]
      (with-translation [[xx yy]]
        (with-rotation [(Math/toRadians (- 180 h))]
          (triangle 0 0
                    -2 7
                    2 7))))))

(defn show-robot [robot maze]
  (fill-int (color 0 255 100))
  (let [{x :x y :y} robot
        s (:one-px maze)]
    (ellipse (* s x) (* s y) 10 10 )))

(defn weight-to-color [weight]
  ;; todo
  )

(defn is-in [maze x y]
  (not (or (< x 0)
           (< y 0)
           (> x (:width maze))
           (> y (:height maze)))))

(defn getxy [maze x y]
  (nth
   (nth (:maze maze) (int y))
   (int x)))

(defn is-free [maze x y]
  (if (not (is-in maze x y))
    false
    (== (getxy maze
               x
               (- (:height maze) y))
        0)))

(defn random-place [maze]
  [(rand (:width maze))
   (rand (:height maze))])

(defn random-free-place [maze]
  (let [pos (random-place maze)]
    (if (is-free maze (first pos) (second pos))
      pos
      (recur maze))))

(defn distance-to-nearest-beacon [maze x y]
  (letfn [(dist [[beacon-x beacon-y]]
            (Math/sqrt
             (+ (Math/pow (- x beacon-x) 2)
                (Math/pow (- y beacon-y) 2))))]
    (apply min
           (map dist (:beacons maze)))))





