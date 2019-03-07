;---------------------------------------------------------------------------
;  fawkes-battery.clp - CEDAR Fawkes battery monitoring
;
;  Created: Tue Jan 21 18:45:10 2014
;  Copyright  2011-2014  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file of cedar
;---------------------------------------------------------------------------

(defglobal
  ?*FAWKES-BATTERY-SOC-LOW*  = 0.5
  ?*FAWKES-BATTERY-SOC-CRIT* = 0.3

  ?*FAWKES-BATTERY-VOLTAGE-LOW*  = 0.5
  ?*FAWKES-BATTERY-VOLTAGE-CRIT* = 0.3
)

(deffacts fawkes-battery-facts
  (system-condition (id fawkes-battery))
)

(defrule fawkes-battery-config-voltage-low
  (confval (path "/cedar/fawkes/battery/voltage-low") (type FLOAT) (value ?v))
  =>
  (bind ?*FAWKES-BATTERY-VOLTAGE-LOW* ?v)
)
  
(defrule fawkes-battery-config-voltage-crit
  (confval (path "/cedar/fawkes/battery/voltage-crit") (type FLOAT) (value ?v))
  =>
  (bind ?*FAWKES-BATTERY-VOLTAGE-CRIT* ?v)
)
  
(defrule fawkes-battery-config-soc-low
  (confval (path "/cedar/fawkes/battery/soc-low") (type FLOAT) (value ?v))
  =>
  (bind ?*FAWKES-BATTERY-SOC-LOW* ?v)
)
  
(defrule fawkes-battery-config-soc-crit
  (confval (path "/cedar/fawkes/battery/soc-crit") (type FLOAT) (value ?v))
  =>
  (bind ?*FAWKES-BATTERY-SOC-CRIT* ?v)
)

(defrule fawkes-battery-init
  (cedar-init)
  (confval (path "/cedar/fawkes/battery/interfaces") (type STRING) (is-list TRUE) (list-value $?v))
  =>
  (foreach ?id ?v (blackboard-open "BatteryInterface" ?id))
)

(defrule fawkes-battery-init
  (cedar-init)
  (confval (path "/cedar/fawkes/battery/interfaces") (type STRING) (is-list TRUE) (list-value $?v))
  =>
  (foreach ?id ?v (blackboard-open "BatteryInterface" ?id))
)

(deffunction add (?a ?b)
  (return (+ ?a ?b))
)

(deffunction add (?a ?b)
  (return (+ ?a ?b))
)

(defrule fawkes-battery-syscond
  (fawkes-collect-process $?now)
  ?cf <- (system-condition (id fawkes-battery) (updated $?u&:(neq ?u ?now)))
  =>
  (bind ?cond GREEN)
  (bind ?desc (create$))

  ; Note that the ordering of the tests is important, first check for
  ; yellow conditions, and only then for red.

  (if (not (any-factp ((?batif BatteryInterface) (?info blackboard-interface-info))
		      (and (eq ?info:type "BatteryInterface") (eq ?batif:id ?info:id)
			   (eq ?info:has-writer TRUE))))
   then
    (bind ?cond YELLOW)
    (bind ?desc (append$ ?desc "No BatteryInterface with a writer exists"))
  )

  ; YELLOW
  (do-for-all-facts ((?batif BatteryInterface))
		    (and (<> ?batif:relativ_soc 0.0) (< ?batif:relativ_soc ?*FAWKES-BATTERY-SOC-LOW*))
    (bind ?cond YELLOW)
    (bind ?desc (append$ ?desc (str-cat "Battery " ?batif:id " charge is low")))
  )

  (do-for-all-facts ((?batif BatteryInterface))
		    (and (<> ?batif:voltage 0.0) (< ?batif:voltage ?*FAWKES-BATTERY-VOLTAGE-LOW*))
    (bind ?cond YELLOW)
    (bind ?desc (append$ ?desc (str-cat "Battery " ?batif:id " voltage is low")))
  )

  ; RED
  (do-for-all-facts ((?batif BatteryInterface))
		    (and (<> ?batif:relativ_soc 0.0) (< ?batif:relativ_soc ?*FAWKES-BATTERY-SOC-CRIT*))
    (bind ?cond RED)
    (bind ?desc (append$ ?desc (str-cat "Battery " ?batif:id " charge is critical")))
  )

  (do-for-all-facts ((?batif BatteryInterface))
		    (and (<> ?batif:voltage 0.0) (< ?batif:voltage ?*FAWKES-BATTERY-VOLTAGE-CRIT*))
    (bind ?cond RED)
    (bind ?desc (append$ ?desc (str-cat "Battery " ?batif:id " voltage is critical")))
  )

  (modify ?cf (cond ?cond) (desc (syscond-join-desc ?desc)) (updated ?now))
)
