
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Pipe } from '@angular/core';

@Pipe({name: 'default', pure: true})
export class DefaultPipe {
  transform(value: any, defaultValue: any): any {
    return value || defaultValue;
  }
}

