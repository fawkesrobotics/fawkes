
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Injectable } from '@angular/core';
import { MatSnackBar } from '@angular/material';
import { SwUpdate } from '@angular/service-worker';
import { Observable, interval } from 'rxjs';
import { environment } from '../../environments/environment';

@Injectable()
export class SwUpdateNotifierService {

  constructor(private updates: SwUpdate,
              private snack_bar: MatSnackBar)
  {
    updates.available.subscribe(event => {
      console.log('current version is', event.current);
      console.log('available version is', event.available);
      let ref = snack_bar.open('Webview Update Available', 'Reload');
      ref.onAction().subscribe(() => {
        updates.activateUpdate().then(() => document.location.reload());
      });
    });
    /*
    updates.activated.subscribe(event => {
      console.log('old version was', event.previous);
      console.log('new version is', event.current);
    });
    */

    if (environment.production) {
      this.force_install();
      interval(60000).subscribe((num) => {
        this.check_now();
      });
    }
  }

  check_now()
  {
    this.updates.checkForUpdate();
  }

  force_install()
  {
    if ('serviceWorker' in navigator) {
      navigator.serviceWorker.register('/ngsw-worker.js').then(function (registration) {
        //console.log('Service Worker registered');
      }).catch(function (err) {
        console.error('Service Worker registration failed: ', err);
      });
    } else {
      console.warn("No service worker");
    }
  }
}
