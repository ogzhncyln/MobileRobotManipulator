;; =============================================================================
;; MRM - Mobil Manipülatör Renk Ayıklama Domain'i (PlanSys2 / POPF)
;; =============================================================================
;; Bu dosya robotun DÜNYAYI NASIL GÖRDÜĞÜNÜ (predicate'ler) ve NELER
;; YAPABİLDİĞİNİ (action'lar) sembolik olarak tanımlar. Sayısal geometri yok:
;; "robot room1'de", "red_room1_0 kırmızı", "door_12 kapalı" gibi doğru/yanlış
;; önermeler var. Planlayıcı (POPF), başlangıç durumundan hedefe giden action
;; dizisini bu tanımlardan çıkarır.
;;
;; Durative action: her action'ın süresi ve koşullarının NE ZAMAN geçerli
;; olması gerektiği vardır:
;;   at start  -> action başlarken
;;   over all  -> action boyunca
;;   at end    -> action biterken
;; PlanSys2 executor bu koşulları ÇALIŞMA ANINDA da problem_expert'e karşı
;; kontrol eder (örn. battery_ok at start). Batarya kesintisi bu sayede çalışır.
;;
;; robot_free: Tek robot aynı anda tek iş yapar. PDDL bunu kendiliğinden bilmez;
;; zamansal planlayıcı (POPF) çakışmayan action'ları PARALEL planlar (testte
;; 'open_door' ile 'pick' t=0'da birlikte başlatıldı ve beceri sunucusu ikincisini
;; reddetti). Her action başlarken robot_free'yi alır, biterken geri verir.
;; =============================================================================

(define (domain mrm_sort)
  (:requirements :strips :typing :durative-actions)

  (:types
    robot room door piece bin color dock
  )

  (:predicates
    (robot_at ?r - robot ?rm - room)              ; robot hangi odada
    (connected ?from - room ?to - room ?d - door) ; iki oda bir kapıyla bağlı
    (button_side ?d - door ?rm - room)            ; kapının butonu bu odada
    (door_open ?d - door)
    (door_closed ?d - door)
    (scanned ?rm - room)                          ; oda kamerayla tarandı
    (piece_at ?p - piece ?rm - room)              ; parça yerde, bu odada
    (piece_color ?p - piece ?c - color)
    (bin_at ?b - bin ?rm - room)
    (bin_color ?b - bin ?c - color)
    (holding ?r - robot ?p - piece)
    (hand_free ?r - robot)
    (in_bin ?p - piece ?b - bin)                  ; parça kutuda (hedef)
    (dock_at ?dk - dock ?rm - room)
    (battery_ok ?r - robot)                       ; batarya yeterli
    (robot_free ?r - robot)                       ; robot boşta (aynı anda tek action kilidi)
  )

  ;; ---------------------------------------------------------------------------
  ;; Odalar arası hareket (Nav2). Kapı açık olmalı.
  ;; battery_ok GEREKMEZ: düşük bataryada da dock'a dönebilmeli.
  (:durative-action move
    :parameters (?r - robot ?from - room ?to - room ?d - door)
    :duration (= ?duration 40)
    :condition (and
      (at start (robot_free ?r))
      (at start (robot_at ?r ?from))
      (over all (connected ?from ?to ?d))
      (over all (door_open ?d)))
    :effect (and
      (at start (not (robot_free ?r)))
      (at end (robot_free ?r))
      (at start (not (robot_at ?r ?from)))
      (at end (robot_at ?r ?to))))

  ;; ---------------------------------------------------------------------------
  ;; Odayı kamerayla tara (Nav2 Spin + renk algılama).
  (:durative-action scan
    :parameters (?r - robot ?rm - room)
    :duration (= ?duration 30)
    :condition (and
      (at start (robot_free ?r))
      (at start (battery_ok ?r))
      (over all (robot_at ?r ?rm)))
    :effect (and
      (at start (not (robot_free ?r)))
      (at end (robot_free ?r))
      (at end (scanned ?rm))))

  ;; ---------------------------------------------------------------------------
  ;; Yerdeki parçayı al (Nav2 yaklaşma + kamera + IK + MoveIt + gripper).
  (:durative-action pick
    :parameters (?r - robot ?p - piece ?rm - room)
    :duration (= ?duration 60)
    :condition (and
      (at start (robot_free ?r))
      (at start (battery_ok ?r))
      (at start (hand_free ?r))
      (at start (piece_at ?p ?rm))
      (at start (scanned ?rm))
      (over all (robot_at ?r ?rm)))
    :effect (and
      (at start (not (robot_free ?r)))
      (at end (robot_free ?r))
      (at start (not (hand_free ?r)))
      (at start (not (piece_at ?p ?rm)))
      (at end (holding ?r ?p))))

  ;; ---------------------------------------------------------------------------
  ;; Tutulan parçayı AYNI RENKTEKİ kutuya bırak.
  ;; Renk eşleşmesi tamamen sembolik: piece_color ve bin_color aynı ?c olmalı.
  (:durative-action place
    :parameters (?r - robot ?p - piece ?b - bin ?rm - room ?c - color)
    :duration (= ?duration 40)
    :condition (and
      (at start (robot_free ?r))
      (at start (holding ?r ?p))
      (at start (piece_color ?p ?c))
      (at start (bin_color ?b ?c))
      (over all (bin_at ?b ?rm))
      (over all (robot_at ?r ?rm)))
    :effect (and
      (at start (not (robot_free ?r)))
      (at end (robot_free ?r))
      (at start (not (holding ?r ?p)))
      (at end (in_bin ?p ?b))
      (at end (hand_free ?r))))

  ;; ---------------------------------------------------------------------------
  ;; Kapının sağındaki butona kolla bas, kapı açılsın.
  (:durative-action open_door
    :parameters (?r - robot ?d - door ?rm - room ?other - room)
    :duration (= ?duration 50)
    :condition (and
      (at start (robot_free ?r))
      (at start (battery_ok ?r))
      (at start (door_closed ?d))
      (at start (hand_free ?r))
      (over all (button_side ?d ?rm))
      (over all (connected ?rm ?other ?d))
      (over all (robot_at ?r ?rm)))
    :effect (and
      (at start (not (robot_free ?r)))
      (at end (robot_free ?r))
      (at start (not (door_closed ?d)))
      (at end (door_open ?d))))

  ;; ---------------------------------------------------------------------------
  ;; Şarj istasyonuna yanaş ve bataryayı doldur.
  (:durative-action charge
    :parameters (?r - robot ?dk - dock ?rm - room)
    :duration (= ?duration 80)
    :condition (and
      (at start (robot_free ?r))
      (over all (dock_at ?dk ?rm))
      (over all (robot_at ?r ?rm)))
    :effect (and
      (at start (not (robot_free ?r)))
      (at end (robot_free ?r))
      (at end (battery_ok ?r))))
)
