#!/usr/bin/env python3

# Copyright 2022 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

import boto3
from botocore.config import Config


def gather_list_of_images(ecr_client):
    images = []
    response = ecr_client.describe_repositories(registryId="080653068785")
    for repo in response['repositories']:
        images.append(repo['repositoryName'])
    return images


def gather_untagged_images(ecr_client, images):
    results = {}
    for image in images:
        empty_tags = []
        image_versions = ecr_client.describe_images(repositoryName=image)
        for instance in image_versions["imageDetails"]:
            if "imageTags" not in instance.keys():
                empty_tags.append({"imageDigest":instance["imageDigest"]})
        if empty_tags:
            results[image] = empty_tags
    return results


def delete_images(ecr_client, image_data):
    for image, empty_tag_list in image_data.items():
        print(f"From image: {image} removing images:\n{empty_tag_list}")
        ecr_client.batch_delete_image(repositoryName=image, imageIds=empty_tag_list)


if __name__ == "__main__":
    config = Config(region_name="us-east-1")
    ecr_client = boto3.client('ecr-public',config=config)
    images = gather_list_of_images(ecr_client)
    empty_tags = gather_untagged_images(ecr_client, images)
    delete_images(ecr_client, empty_tags)
