
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Pipe, PipeTransform } from '@angular/core';

@Pipe({name: 'default', pure: true})
export class DefaultPipe implements PipeTransform {
  transform(value: any, defaultValue: any): any {
    return value || defaultValue;
  }
}

