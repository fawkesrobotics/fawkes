; Emacs Editor settings
; For code style compliance, additionally the smart-tabs mode is required.
; https://www.emacswiki.org/emacs/SmartTabs

; This replicates the enforced clang-format as close as possible.
((nil . ((fill-column . 100) (require-final-newline . t) (tab-width . 2)))
 (c-mode . ((c-file-style . "GNU")
						(indent-tabs-mode . t)
						(c-file-offsets
						 (arglist-intro . 2)
						 (member-init-intro . 0)
						 (innamespace . 0)
						 (substatement-open . 0)
						 (statement-cont . '(c-lineup-assignments 2))
						 (arglist-cont-nonempty . '(c-lineup-string-cont
																				c-lineup-gcc-asm-reg c-lineup-arglist))
						 )
						))
)
