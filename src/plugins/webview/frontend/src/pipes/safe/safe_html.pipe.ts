
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Pipe } from '@angular/core';
import { DomSanitizer } from '@angular/platform-browser';

@Pipe({name: 'safe_html'})
export class SafeHTMLPipe {
  constructor(private sanitizer: DomSanitizer){}

  transform(value: any): any {
    return this.sanitizer.bypassSecurityTrustHtml(value);
  }
}
