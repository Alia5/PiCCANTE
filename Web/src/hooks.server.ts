/* eslint-disable no-return-await */
import { mocks as settingMocks } from '$mocks/settings.server';
import { mocks as aboutMocks } from '$mocks/about.server';
export const handle = async ({ event, resolve }) => {

    const mocks = {
        ...aboutMocks,
        ...settingMocks
    };

    const mock = mocks?.[event.url.pathname];
    if (mock) {
        const method = event.request.method;
        const response = mock[method];
        if (response) {
            if (response.func) {
                response.func(await event.request.json());
            }
            return new Response(JSON.stringify(response.body), {
                status: response.status,
                headers: {
                    'Content-Type': 'application/json'
                }
            });
        } else {
            return new Response('Method Not Allowed', { status: 405 });
        }
    }

    return await resolve(event);
};
