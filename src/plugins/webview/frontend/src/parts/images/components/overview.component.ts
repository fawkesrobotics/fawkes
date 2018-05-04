
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Component, OnInit, OnDestroy, ViewChild, HostListener, ElementRef } from '@angular/core';
import { Observable, interval } from 'rxjs';

import { ImageApiService } from '../services/api.service';
import { ImageInfo } from '../models/ImageInfo';

import { BackendConfigurationService } from '../../../services/backend-config/backend-config.service';

@Component({
  selector: 'blackboard-overview',
  templateUrl: './overview.component.html',
  styleUrls: ['./overview.component.scss']
})
export class ImageOverviewComponent implements OnInit, OnDestroy {


  /* Why do we need this reference to a div?
   * Ideally, we would have an img with a bound src attribute, like
   * this:
   * <img [src]="image_src">
   * Then, all we need to do is set image_src and the change detection
   * will take care of the rest. This does work in general, but fails
   * to properly sever on-going mjpeg streams. When switching to
   * another window (blur event) or to another component
   * (ngOnDestroy), setting the image_src to another URL seems to be
   * ignored by the browser (on Chrome 65 this is). The transfer goes
   * on in the background and produces load on the image producer.
   *
   * The workaround is to get a div which will be the parent of the
   * image.  On selection, the img element is created and setup and
   * all is well.  On deselection, we explicitly reset the src and
   * completely remove the img element from the DOM. This, in fact,
   * does stop the transfer reliably.
   */
  @ViewChild('image')
  image_div: ElementRef;

  loading = false;
  auto_refresh_subscription = null;
  zero_message = "No graph has been retrieved";

  images: ImageInfo[] = null;

  image_src: string = null;
  image_selected: string[] = null;
  image_on_blur: string[] = null;

  private backend_subscription = null;

  constructor(private api_service: ImageApiService,
              private backendcfg: BackendConfigurationService)
  {}

  ngOnInit() {
    this.refresh();
    this.image_on_blur = null;
    this.backend_subscription = this.backendcfg.backend_changed.subscribe((b) => {
      this.disable_autorefresh();
      this.deselect_image();
      this.refresh();
    });
  }

  ngOnDestroy()
  {
    this.disable_autorefresh();
    this.deselect_image();
    this.backend_subscription.unsubscribe();
    this.backend_subscription = null;
  }

  @HostListener('window:focus', ['$event'])
  onFocus(ev: FocusEvent)
  {
    if (this.image_on_blur) {
      this.select_image(this.image_on_blur[0], this.image_on_blur[1]);
      this.image_on_blur = null;
    }
  }

  @HostListener('window:blur', ['$event'])
  onBlur(ev: FocusEvent)
  {
    this.image_on_blur = this.image_selected;
    this.deselect_image();
  }

  select_image(image: string, mode: string)
  {
    if (this.image_src)  this.deselect_image();
    
    this.image_selected = [image, mode];
    let cache_bust = `${Date.now()}-${Math.random()}`;
    this.image_src = `${this.backendcfg.url_for('api')}/images/${image}.${mode}?cb=${cache_bust}`;

    // see @ViewChild comment why we do this
    let img_elem = document.createElement("img");
    img_elem.src = this.image_src;
    this.image_div.nativeElement.appendChild(img_elem);
  }

  deselect_image()
  {
    this.image_src = null;
    this.image_selected = null;
    // see @ViewChild comment why we do this
    if (this.image_div.nativeElement.childNodes.length > 0) {
      this.image_div.nativeElement.children[0].src = '';
      this.image_div.nativeElement.removeChild(this.image_div.nativeElement.childNodes[0]);
    }
  }

  refresh()
  {
    this.loading = true;
    this.zero_message = "Retrieving image info";

    this.api_service.list_images().subscribe(
      (images) => {
        this.images = images;
        if (this.images.length == 0) {
          this.zero_message = 'No images available';
        }
        this.loading = false;
      },
      (err) => {
        this.images = null;
        if (err.status == 0) {
          this.zero_message="API server unavailable. Robot down?";
        } else {
          this.zero_message=`Failed to retrieve images: ${err.error}`;
        }
        this.loading = false;
      }
    );
  }

  private enable_autorefresh()
  {
    if (this.auto_refresh_subscription)  return;
    this.auto_refresh_subscription =
      interval(2000).subscribe((num) => {
        this.refresh();
      });
    this.refresh();
  }

  private disable_autorefresh()
  {
    if (this.auto_refresh_subscription) {
      this.auto_refresh_subscription.unsubscribe();
      this.auto_refresh_subscription = null;
    }
  }

  toggle_autorefresh()
  {
    if (this.auto_refresh_subscription) {
      this.disable_autorefresh();
    } else {
      this.enable_autorefresh();
    }
  }
}
