; Copyright (C) 2021 Till Hofmann <hofmann@kbsg.rwth-aachen.de>
; This program is free software; you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation; either version 2 of the License, or
; (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
; GNU General Public License for more details.
;
; You should have received a copy of the GNU General Public License
; along with this program; if not, see <http://www.gnu.org/licenses/>.

(add-to-list 'load-path (concat (getenv "FAWKES_BASEDIR") "/etc/format-scripts/emacs-lisp"))

(add-to-list 'auto-mode-alist '("\\.h\\'" . c++-mode))

(setq require-final-newline t)
;(setq indent-tabs-mode t)

(require 'smart-tabs-mode)
(smart-tabs-insinuate 'c 'c++)

(defun c-offsets ()
  (c-set-offset 'arglist-intro 2)
  (c-set-offset 'member-init-intro 0)
  (c-set-offset 'innamespace 0)
  (c-set-offset 'substatement-open 0)
	(c-set-offset 'statement-cont '(c-lineup-assignments 2))
	(c-set-offset 'arglist-cont-nonempty '(c-lineup-string-cont c-lineup-gcc-asm-reg c-lineup-arglist))
  ; Broken by smart-tabs
	;(c-set-offset 'access-label '/)
)
(add-hook 'c-mode-common-hook 'c-offsets)

(defun format-code ()
	(interactive)
	(c++-mode)
	;(indent-region (point-min) (point-max) nil)
	(mark-whole-buffer)
	;(c-indent-line-or-region (point-min) (point-max))
	(c-indent-line-or-region 0 (buffer-size))
	;(tabify (point-min) (point-max))
	(save-buffer)
)
